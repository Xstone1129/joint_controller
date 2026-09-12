#include "joint_hardware/lift/cia402.hpp"
#include "joint_hardware/lift/ethercat_backend.hpp"
#include "joint_hardware/lift/etherlab_backend.hpp"
#include "joint_hardware/lift/lift_units.hpp"
#include "joint_hardware/lift/pdo_mapping.hpp"
#include "joint_hardware/lift/zero_offset_store.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <cstdio>
#include <pwd.h>
#include <sys/select.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace joint_hardware::lift;

namespace {
std::atomic<bool> running{true};
void stop_signal(int) { running.store(false); }

class TerminalRawMode {
public:
  TerminalRawMode() {
    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &original_) != 0) return;
    original_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (original_flags_ < 0) return;
    termios raw = original_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0 &&
      fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK) == 0) active_ = true;
  }
  ~TerminalRawMode() {
    if (active_) {
      (void)tcsetattr(STDIN_FILENO, TCSANOW, &original_);
      (void)fcntl(STDIN_FILENO, F_SETFL, original_flags_);
    }
  }
  bool active() const noexcept { return active_; }
private:
  termios original_{};
  int original_flags_{-1};
  bool active_{false};
};

std::string trim(std::string s) {
  const auto first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  const auto last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1);
}

std::string system_master_interface(uint32_t master) {
  const std::string key = "MASTER" + std::to_string(master) + "_DEVICE=";
  for (const char * path : {"/usr/local/etherlab/etc/ethercat.conf", "/usr/local/etherlab/etc/sysconfig/ethercat"}) {
    std::ifstream file(path);
    std::string line;
    while (std::getline(file, line)) {
      line = trim(line);
      if (line.rfind(key, 0) != 0) continue;
      auto value = trim(line.substr(key.size()));
      if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
        value = value.substr(1, value.size() - 2);
      }
      if (!value.empty()) return value;
    }
  }
  return {};
}

std::map<std::string, std::string> workspace_lift_config() {
  std::map<std::string, std::string> values;
  const char * paths[] = {
    "/home/user/joint_controller/src/joint_hardware/config/lift_hardware.yaml",
    "src/joint_hardware/config/lift_hardware.yaml"};
  for (const char * path : paths) {
    std::ifstream file(path);
    if (!file) continue;
    std::string line;
    while (std::getline(file, line)) {
      const auto comment = line.find('#');
      if (comment != std::string::npos) line.resize(comment);
      const auto colon = line.find(':');
      if (colon == std::string::npos) continue;
      auto key = trim(line.substr(0, colon));
      auto value = trim(line.substr(colon + 1));
      if (!key.empty() && !value.empty()) values[key] = value;
    }
    break;
  }
  return values;
}

uint32_t config_uint(const std::map<std::string, std::string> & config, const std::string & key, uint32_t fallback) {
  try { return config.count(key) ? static_cast<uint32_t>(std::stoul(config.at(key), nullptr, 0)) : fallback; }
  catch (...) { return fallback; }
}
double config_double(const std::map<std::string, std::string> & config, const std::string & key, double fallback) {
  try { return config.count(key) ? std::stod(config.at(key)) : fallback; }
  catch (...) { return fallback; }
}
std::string config_string(
  const std::map<std::string, std::string> & config, const std::string & key,
  const std::string & fallback)
{
  const auto it = config.find(key);
  return it == config.end() ? fallback : it->second;
}

std::string state_name(Cia402State state) {
  switch (state) {
    case Cia402State::not_ready_to_switch_on: return "not_ready";
    case Cia402State::switch_on_disabled: return "switch_on_disabled";
    case Cia402State::ready_to_switch_on: return "ready_to_switch_on";
    case Cia402State::switched_on: return "switched_on";
    case Cia402State::operation_enabled: return "operation_enabled";
    case Cia402State::quick_stop_active: return "quick_stop_active";
    case Cia402State::fault_reaction_active: return "fault_reaction_active";
    case Cia402State::fault: return "fault";
    default: return "unknown";
  }
}

std::string link_state_name(EthercatLinkState state) {
  switch (state) {
    case EthercatLinkState::offline: return "offline";
    case EthercatLinkState::init: return "init";
    case EthercatLinkState::pre_operational: return "pre_operational";
    case EthercatLinkState::safe_operational: return "safe_operational";
    case EthercatLinkState::operational: return "operational";
  }
  return "invalid";
}

std::string error_name(uint16_t code) {
  if (code == 0) return "no error";
  const std::map<uint16_t, std::string> known{{0x2310, "over-current"}, {0x3110, "over-voltage"},
    {0x3210, "under-voltage"}, {0x4210, "over-temperature"}, {0x7300, "encoder fault"}};
  const auto it = known.find(code);
  return it == known.end() ? "vendor-specific/see drive manual" : it->second;
}

bool discover_identity(uint32_t master, uint16_t position, uint32_t & vendor, uint32_t & product) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (std::chrono::steady_clock::now() < deadline) {
    vendor = 0;
    product = 0;
    std::ostringstream command;
    command << "ethercat slaves -m " << master << " -v 2>/dev/null";
    FILE * pipe = popen(command.str().c_str(), "r");
    if (pipe != nullptr) {
      std::string text; char buffer[512];
      while (fgets(buffer, sizeof(buffer), pipe) != nullptr) text += buffer;
      const int status = pclose(pipe);
      if (status == 0) {
        std::istringstream lines(text); std::string line; bool selected = false;
        while (std::getline(lines, line)) {
          const bool header = line.find("=== Master") != std::string::npos;
          if (header && line.find("Slave " + std::to_string(position)) != std::string::npos) selected = true;
          else if (header && selected) break;
          if (!selected) continue;
          auto parse = [&](const char * label, uint32_t & out) {
            const auto at = line.find(label); if (at == std::string::npos) return false;
            auto value = trim(line.substr(at + std::strlen(label)));
            try { out = std::stoul(value, nullptr, 0); return true; } catch (...) { return false; }
          };
          if (parse("Vendor ID:", vendor) || parse("Vendor Id:", vendor)) {}
          if (parse("Product code:", product)) {}
          if (vendor != 0 && product != 0) return true;
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  return false;
}

struct Options {
  std::string interface{"enp5s0"};
  std::string log_path{"lift_ethercat_cli.csv"};
  bool log_supplied{false};
  std::string zero_offset_file;
  std::string motor_id{"LVM08008H3G3-M17"};
  uint32_t zero_offset_uid{std::numeric_limits<uint32_t>::max()};
  uint32_t zero_offset_gid{std::numeric_limits<uint32_t>::max()};
  uint32_t master{2};
  uint16_t alias{0};
  uint16_t position{0};
  uint32_t vendor{0};
  uint32_t product{0};
  double min_m{-1.0};
  double max_m{0.0};
  double lead_mm{10.0 / 3.0};
  double sign{-1.0};
  uint32_t units{10000};
  double max_speed_mps{0.015};
  double acceleration_mps2{0.033333333};
  bool auto_enable{false};
  bool command_mode{false};
  bool interface_supplied{false};
  bool master_supplied{false};
  bool alias_supplied{false};
  bool position_supplied{false};
  bool vendor_supplied{false};
  bool product_supplied{false};
  bool min_supplied{false};
  bool max_supplied{false};
};

void usage(const char * name) {
  std::cout << "Usage: " << name << " [--enable] [--command-mode] [--interface IF] [--master N] [--alias N] [--position N] [--vendor ID] [--product ID] [--min-position M] [--max-position M] [--speed MPS] [--accel MPS2] [--zero-offset-file PATH] [--zero-offset-uid UID] [--zero-offset-gid GID] [--log PATH]\n"
            << "Keys: e enable/hold, d disable, z zero, up/down arrows +/-100 mm task, hold u/j jog +/- coordinate, space stop, q quit\n";
}
}  // namespace

int main(int argc, char ** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--enable") options.auto_enable = true;
    else if (arg == "--command-mode") options.command_mode = true;
    else if (arg == "--interface" && i + 1 < argc) { options.interface = argv[++i]; options.interface_supplied = true; }
    else if (arg == "--log" && i + 1 < argc) { options.log_path = argv[++i]; options.log_supplied = true; }
    else if (arg == "--master" && i + 1 < argc) { options.master = std::stoul(argv[++i], nullptr, 0); options.master_supplied = true; }
    else if (arg == "--alias" && i + 1 < argc) { options.alias = static_cast<uint16_t>(std::stoul(argv[++i], nullptr, 0)); options.alias_supplied = true; }
    else if (arg == "--position" && i + 1 < argc) { options.position = static_cast<uint16_t>(std::stoul(argv[++i], nullptr, 0)); options.position_supplied = true; }
    else if (arg == "--vendor" && i + 1 < argc) { options.vendor = std::stoul(argv[++i], nullptr, 0); options.vendor_supplied = true; }
    else if (arg == "--product" && i + 1 < argc) { options.product = std::stoul(argv[++i], nullptr, 0); options.product_supplied = true; }
    else if (arg == "--min-position" && i + 1 < argc) { options.min_m = std::stod(argv[++i]); options.min_supplied = true; }
    else if (arg == "--max-position" && i + 1 < argc) { options.max_m = std::stod(argv[++i]); options.max_supplied = true; }
    else if (arg == "--speed" && i + 1 < argc) options.max_speed_mps = std::stod(argv[++i]);
    else if (arg == "--accel" && i + 1 < argc) options.acceleration_mps2 = std::stod(argv[++i]);
    else if (arg == "--zero-offset-file" && i + 1 < argc) options.zero_offset_file = argv[++i];
    else if (arg == "--zero-offset-uid" && i + 1 < argc) options.zero_offset_uid = std::stoul(argv[++i], nullptr, 0);
    else if (arg == "--zero-offset-gid" && i + 1 < argc) options.zero_offset_gid = std::stoul(argv[++i], nullptr, 0);
    else if (arg == "--help") { usage(argv[0]); return 0; }
  }

  std::signal(SIGINT, stop_signal); std::signal(SIGTERM, stop_signal);
  const auto config = workspace_lift_config();
  if (!options.master_supplied) options.master = config_uint(config, "ethercat_master_index", options.master);
  if (!options.alias_supplied) options.alias = static_cast<uint16_t>(config_uint(config, "slave_alias", options.alias));
  if (!options.position_supplied) options.position = static_cast<uint16_t>(config_uint(config, "slave_position", options.position));
  if (!options.vendor_supplied) options.vendor = config_uint(config, "slave_vendor_id", options.vendor);
  if (!options.product_supplied) options.product = config_uint(config, "slave_product_code", options.product);
  if (!options.min_supplied) options.min_m = config_double(config, "position_min_m", options.min_m);
  if (!options.max_supplied) options.max_m = config_double(config, "position_max_m", options.max_m);
  options.lead_mm = config_double(config, "lead_mm_per_rev", options.lead_mm);
  options.sign = config_double(config, "lift_sign", options.sign);
  options.units = config_uint(config, "command_units_per_rev", options.units);
  options.motor_id = config_string(config, "motor_id", options.motor_id);
  if (!options.log_supplied) {
    const char * dynamic_session = std::getenv("JUNIOR_RUNTIME_LOG_DYNAMIC");
    if (const char * root = std::getenv("JOINT_CONTROLLER_LOG_ROOT");
        root != nullptr && *root != '\0' &&
        !(dynamic_session != nullptr && std::string(dynamic_session) == "1")) {
      options.log_path = std::string(root) + "/lift_ethercat_cli.csv";
    } else {
      const char * configured_user = std::getenv("JUNIOR_RUNTIME_LOG_USER");
      const char * sudo_user = std::getenv("SUDO_USER");
      const passwd * account = nullptr;
      if (configured_user != nullptr && *configured_user != '\0') {
        account = getpwnam(configured_user);
      } else if (sudo_user != nullptr && *sudo_user != '\0') {
        account = getpwnam(sudo_user);
      } else {
        account = getpwuid(getuid());
        if (account != nullptr && std::string(account->pw_name) == "root") {
          account = getpwnam("user");
        }
      }
      const std::string home = account != nullptr ? account->pw_dir :
        (std::getenv("HOME") != nullptr ? std::getenv("HOME") : "/home/user");
      std::string session = "standalone-hardware";
      const char * marker_override = std::getenv("JOINT_CONTROLLER_SESSION_NAME_FILE");
      const std::string marker_path = marker_override != nullptr && *marker_override != '\0' ?
        marker_override : home + "/joint_controller/.junior_runtime_session_name";
      std::ifstream marker(marker_path);
      std::string candidate;
      if (marker && std::getline(marker, candidate) &&
        !candidate.empty() && candidate.find_first_not_of(
          "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_.-") == std::string::npos)
      {
        session = candidate;
      }
      const char * log_root = std::getenv("JUNIOR_LOG_ROOT");
      const std::string log_root_path = log_root != nullptr && *log_root != '\0' ?
        log_root : home + "/.ros/log";
      options.log_path = log_root_path + "/" + session + "/runtime/lift_ethercat_cli.csv";
    }
  }
  if (!std::isfinite(options.max_speed_mps) || options.max_speed_mps <= 0.0 ||
    !std::isfinite(options.acceleration_mps2) || options.acceleration_mps2 <= 0.0) {
    std::cerr << "--speed and --accel must be positive finite values.\n";
    return 2;
  }
  if (!std::isfinite(options.min_m) || !std::isfinite(options.max_m) ||
    options.min_m >= options.max_m) {
    std::cerr << "--min-position must be less than --max-position.\n";
    return 2;
  }
  std::ofstream trace(options.log_path, std::ios::out | std::ios::trunc);
  if (!trace.is_open()) {
    std::cerr << "无法打开诊断日志: " << options.log_path << "\n";
    return 2;
  }
  trace << "elapsed_ms,status_word,status_mask,error_code,mode_display,digital_inputs,"
           "control_word,state,enable_requested,allow_motion,link_state,pdo_fresh,"
           "working_counter,actual_position_units,actual_velocity_units,"
           "target_velocity_units,target_m,position_m,gate,decision_fault,"
           "decision_health_ok,decision_mode_ok,recovery_exhausted,fault_latched,"
           "latched_fault_code,last_action\n";
  trace.flush();
  if (!options.interface_supplied) {
    if (const char * env = std::getenv("LIFT_ETHERCAT_INTERFACE")) options.interface = env;
    else if (const auto interface = system_master_interface(options.master); !interface.empty()) options.interface = interface;
  }

  if ((options.vendor == 0 || options.product == 0) && !discover_identity(options.master, options.position, options.vendor, options.product)) {
    std::cerr << "无法自动读取 lift 从站身份。请使用 tools/hardware/run_hardware_console.sh lift 启动，"
              << "或确认 /dev/EtherCAT" << options.master << " 存在；也可用 --vendor/--product 覆盖身份。\n";
    return 2;
  }
  std::cout << "发现 lift 从站：vendor=0x" << std::hex << options.vendor
            << " product=0x" << options.product << std::dec << "\n";
  auto backend = make_lift_ethercat_backend("etherlab");
  EthercatMasterConfig master{options.interface, options.master, std::chrono::milliseconds(10), 1};
  EthercatSlaveConfig slave{}; slave.alias = options.alias; slave.position = options.position; slave.vendor_id = options.vendor; slave.product_code = options.product;
  std::cout << "请求 EtherLab Master" << options.master << "..." << std::endl;
  if (!backend->initialize(master)) { std::cerr << "连接 EtherCAT 主站失败: " << backend->error_message() << "\n"; return 2; }
  std::cout << "配置 lift 从站 PDO..." << std::endl;
  if (!backend->configure_slave(slave)) { std::cerr << "配置 lift 从站失败: " << backend->error_message() << "\n"; return 2; }
  std::string pdo_error;
  if (!configure_lift_csv_pdos(*backend, pdo_error) || !backend->write_sdo(0x6060, 0, {9})) {
    std::cerr << "配置 CSV/PDO 失败: " << (pdo_error.empty() ? backend->error_message() : pdo_error) << "\n"; return 2;
  }
  std::cout << "激活 100 Hz PDO 周期..." << std::endl;
  if (!backend->start()) { std::cerr << "启动 EtherCAT 周期失败: " << backend->error_message() << "\n"; return 2; }
  // Command-mode parents use this line as the process readiness handshake.
  // Emit it immediately after the backend starts; waiting for the first PDO
  // iteration made startup depend on pipe scheduling and could time out even
  // while the CLI was already receiving valid PDOs.
  if (options.command_mode) std::cout << "COMMAND_MODE_READY" << std::endl;
  LiftUnitConfig units{}; units.lead_mm_per_rev = options.lead_mm; units.lift_sign = options.sign;
  units.command_units_per_rev = options.units; units.effective_command_units_per_rev = options.units;

  TerminalRawMode terminal;
  if (!options.command_mode && !terminal.active()) {
    std::cerr << "CLI 必须在交互终端中运行。\n";
    (void)backend->stop();
    return 2;
  }
  int command_input_flags = -1;
  if (options.command_mode) {
    command_input_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (command_input_flags < 0 || fcntl(STDIN_FILENO, F_SETFL, command_input_flags | O_NONBLOCK) != 0) {
      std::cerr << "无法配置 command mode 输入管道。\n";
      (void)backend->stop();
      return 2;
    }
  }
  LiftRxPdo output{}; LiftTxPdo input{}; Cia402Controller cia(3, 10);
  int32_t zero = 0;
  std::string zero_offset_status{"host zero offset disabled"};
  if (!options.zero_offset_file.empty()) {
    ZeroOffsetRecord record;
    std::string zero_error;
    const auto loaded = ZeroOffsetStore::load(
      options.zero_offset_file, options.motor_id, options.alias, options.position, record, zero_error);
    if (loaded == ZeroOffsetLoadResult::invalid) {
      std::cerr << "零偏文件无效: " << zero_error << "\n";
      (void)backend->stop();
      return 2;
    }
    if (loaded == ZeroOffsetLoadResult::loaded) {
      zero = record.zero_offset_units;
      zero_offset_status = "loaded " + std::to_string(zero) + " units";
    } else {
      zero_offset_status = "no saved offset";
    }
  }
  bool enable = false; double target = 0.0; bool have_sample = false;
  bool fault_latched = false;
  bool zero_request_pending = false;
  uint16_t latched_fault_code = 0;
  uint32_t enable_wait_cycles = 0;
  double last_joint_rpm = 0.0;
  std::string last_action{"waiting for first PDO"};
  std::string motion_gate{"waiting for first PDO"};
  constexpr double kTaskStepM = 0.100;
  constexpr auto kJogReleaseTimeout = std::chrono::milliseconds(400);
  int jog_direction = 0;
  std::chrono::steady_clock::time_point last_jog_input{};
  auto request_position_task = [&](int direction) {
    const double position = position_units_to_m(input.actual_position_units, zero, units);
    if (fault_latched) {
      last_action = "drive fault is latched; press e after inspection to retry";
      return;
    }
    const double requested = position + static_cast<double>(direction) * kTaskStepM;
    target = std::clamp(requested, options.min_m, options.max_m);
    jog_direction = 0;
    enable = true;
    last_action = target == requested ?
      (direction > 0 ? "up task: +100 mm" : "down task: -100 mm") :
      "100 mm task clamped at coordinate limit";
  };
  auto issue_key = [&](char key) {
    const double position = position_units_to_m(input.actual_position_units, zero, units);
    if (!have_sample && key != 'q' && key != 'Q') return;
    switch (key) {
      case 'e': case 'E':
        fault_latched = false;
        latched_fault_code = 0;
        enable = true;
        target = position;
        last_action = "enabled: holding current position";
        break;
      case 'd': case 'D':
        jog_direction = 0;
        enable = false;
        target = position;
        last_action = "disabled";
        break;
      case 'z': case 'Z':
        if (options.zero_offset_file.empty()) {
          last_action = "zero refused: --zero-offset-file is required";
        } else if (!backend->pdo_fresh() ||
          backend->link_state() != EthercatLinkState::operational ||
          input.error_code != 0U ||
          std::abs(velocity_units_to_mps(input.actual_velocity_units_per_s, units)) > 0.001) {
          last_action = "zero refused: PDO must be fresh, error-free, and stationary";
        } else {
          jog_direction = 0;
          enable = false;
          target = position;
          zero_request_pending = true;
          last_action = "zero requested: controlled disable before saving 6064h offset";
        }
        break;
      case 'u': case 'U': {
        if (fault_latched) {
          last_action = "drive fault is latched; press e after inspection to retry";
          break;
        }
        enable = true;
        jog_direction = 1;
        last_jog_input = std::chrono::steady_clock::now();
        last_action = "jog + coordinate: hold U; release stops";
        break;
      }
      case 'j': case 'J': {
        if (fault_latched) {
          last_action = "drive fault is latched; press e after inspection to retry";
          break;
        }
        enable = true;
        jog_direction = -1;
        last_jog_input = std::chrono::steady_clock::now();
        last_action = "jog - coordinate: hold J; release stops";
        break;
      }
      case ' ': jog_direction = 0; enable = false; target = position; last_action = "stopped and disabled"; break;
      case 'q': case 'Q': running.store(false); break;
      default: break;
    }
  };
  auto issue_command = [&](const std::string & raw) {
    const auto command = trim(raw);
    if (command.empty()) return;
    std::istringstream input_line(command);
    std::string verb; input_line >> verb;
    for (auto & c : verb) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    if (verb == "ENABLE") { issue_key('e'); return; }
    if (verb == "DISABLE") { issue_key('d'); return; }
    if (verb == "ZERO") { issue_key('z'); return; }
    if (verb == "HOME") {
      if (!have_sample) return;
      if (fault_latched) {
        last_action = "home refused: drive fault is latched; press e after inspection to retry";
        return;
      }
      jog_direction = 0;
      enable = true;
      target = 0.0;
      last_action = "home requested: moving to logical zero";
      return;
    }
    if (verb == "HOLD") {
      if (!have_sample) return;
      target = position_units_to_m(input.actual_position_units, zero, units);
      jog_direction = 0;
      last_joint_rpm = 0.0;
      last_action = enable ? "holding current position" : "disabled at current position";
      return;
    }
    if (verb == "STATE") {
      if (!have_sample) {
        std::cout << "LIFT_STATE available=0" << std::endl;
        return;
      }
      const double position = position_units_to_m(input.actual_position_units, zero, units);
      const double velocity = velocity_units_to_mps(input.actual_velocity_units_per_s, units);
      std::cout << "LIFT_STATE available=1"
                << " position_m=" << std::fixed << std::setprecision(9) << position
                << " velocity_mps=" << velocity
                << " pdo_fresh=" << (backend->pdo_fresh() ? 1 : 0)
                << " link_state=" << link_state_name(backend->link_state())
                << " status_word=" << input.status_word
                << " error_code=" << input.error_code
                << std::endl;
      return;
    }
    if (verb == "QUIT") { issue_key('q'); return; }
    if (verb == "MOVE_STEP") {
      double delta = 0.0;
      if (!(input_line >> delta) || !std::isfinite(delta) || std::abs(delta) > 0.1) {
        last_action = "MOVE_STEP refused: delta must be finite and <= 0.1 m";
        return;
      }
      if (!have_sample || fault_latched || !enable) {
        last_action = "MOVE_STEP refused: lift must be enabled and fault-free";
        return;
      }
      const double position = position_units_to_m(input.actual_position_units, zero, units);
      target = std::clamp(position + delta, options.min_m, options.max_m);
      jog_direction = 0; enable = true;
      last_action = "MOVE_STEP target=" + std::to_string(target);
      return;
    }
    last_action = "unknown command: " + command;
  };
  std::cout << "lift EtherCAT CLI 已连接。诊断日志: " << options.log_path << std::endl;
  if (options.auto_enable) { enable = true; last_action = "enable requested"; }
  const auto trace_start = std::chrono::steady_clock::now();
  auto next_render = std::chrono::steady_clock::now();
  auto next_trace = next_render;
  bool have_trace_snapshot = false;
  uint16_t logged_status_word = 0;
  uint16_t logged_error_code = 0;
  uint8_t logged_mode = 0;
  uint32_t logged_digital_inputs = 0;
  uint16_t logged_control_word = 0;
  uint32_t logged_wkc = 0;
  bool logged_enable = false;
  bool logged_allow_motion = false;
  bool logged_fault_latched = false;
  bool logged_pdo_fresh = false;
  EthercatLinkState logged_link = EthercatLinkState::offline;
  std::string logged_gate;
  std::string pending_terminal_input;
  std::string command_input;
  while (running.load()) {
    char keys[32];
    const ssize_t count = read(STDIN_FILENO, keys, sizeof(keys));
    if (count > 0) {
      if (options.command_mode) {
        command_input.append(keys, static_cast<size_t>(count));
        size_t newline = 0;
        while ((newline = command_input.find('\n')) != std::string::npos) {
          issue_command(command_input.substr(0, newline));
          command_input.erase(0, newline + 1);
        }
      } else {
      pending_terminal_input.append(keys, static_cast<size_t>(count));
      while (!pending_terminal_input.empty()) {
        if (pending_terminal_input.front() != '\033') {
          issue_key(pending_terminal_input.front());
          pending_terminal_input.erase(0, 1);
          continue;
        }
        if (pending_terminal_input.size() < 3U) break;
        if (pending_terminal_input[1] == '[' && pending_terminal_input[2] == 'A') {
          request_position_task(1);
          pending_terminal_input.erase(0, 3);
        } else if (pending_terminal_input[1] == '[' && pending_terminal_input[2] == 'B') {
          request_position_task(-1);
          pending_terminal_input.erase(0, 3);
        } else {
          pending_terminal_input.erase(0, 1);
        }
      }
      }
    } else if (count == 0 && options.command_mode) {
      enable = false;
      running.store(false);
    } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      last_action = "terminal read error";
      running.store(false);
    }
    if (!backend->read_pdo(input)) { std::cerr << "PDO 读取失败: " << backend->error_message() << "\n"; break; }
    have_sample = true;
    const auto cycle_now = std::chrono::steady_clock::now();
    if (jog_direction != 0 && cycle_now - last_jog_input > kJogReleaseTimeout) {
      jog_direction = 0;
      target = position_units_to_m(input.actual_position_units, zero, units);
      last_action = "jog released: holding current position";
    }
    if (input.error_code != 0U && !fault_latched) {
      fault_latched = true;
      latched_fault_code = input.error_code;
      enable = false;
      jog_direction = 0;
      last_action = "drive fault latched; inspect drive before pressing e to retry";
    }
    const auto state = parse_cia402_status(input.status_word);
    const bool healthy = backend->pdo_fresh() && backend->link_state() == EthercatLinkState::operational;
    if (enable && healthy && input.error_code == 0U &&
      input.mode_display == 9 && state == Cia402State::switched_on) {
      ++enable_wait_cycles;
    } else {
      enable_wait_cycles = 0;
    }
    Cia402Inputs cia_in{}; cia_in.status_word = input.status_word; cia_in.error_code = input.error_code; cia_in.mode_display = input.mode_display;
    cia_in.link_operational = healthy; cia_in.pdo_fresh = healthy; cia_in.working_counter_ok = backend->working_counter() >= 1; cia_in.request_enable = enable; cia_in.request_fault_reset = true;
    const auto decision = cia.step(cia_in);
    output.control_word = decision.control_word;
    const double position = position_units_to_m(input.actual_position_units, zero, units);
    if (last_action == "waiting for first PDO") { target = position; last_action = "disabled; holding current position"; }
    const double error = target - position;
    const double velocity = velocity_units_to_mps(input.actual_velocity_units_per_s, units);
    // Match the ROS reset-zero sequence: seeing a fresh stationary cycle with
    // the requested controlled-disable word is sufficient.  Some drives
    // remain in Ready to Switch On (0x0021) while 0x0006 is held.
    if (zero_request_pending && decision.control_word == 0x0006 &&
      std::abs(velocity) <= 0.001) {
      ZeroOffsetRecord record;
      record.motor_id = options.motor_id;
      record.slave_alias = options.alias;
      record.slave_position = options.position;
      record.zero_offset_units = input.actual_position_units;
      std::string zero_error;
      if (ZeroOffsetStore::save_atomic(options.zero_offset_file, record, zero_error)) {
        if ((options.zero_offset_uid != std::numeric_limits<uint32_t>::max() ||
          options.zero_offset_gid != std::numeric_limits<uint32_t>::max()) &&
          chown(
            options.zero_offset_file.c_str(), options.zero_offset_uid,
            options.zero_offset_gid) != 0)
        {
          last_action = "zero saved but ownership update failed: " +
            std::string(std::strerror(errno));
        } else {
          zero = record.zero_offset_units;
          target = 0.0;
          zero_offset_status = "saved " + std::to_string(zero) + " units";
          last_action = "zero saved: shared host offset applied";
        }
      } else {
        last_action = "zero save failed: " + zero_error;
      }
      zero_request_pending = false;
    }
    const bool target_reached = std::abs(error) <= 0.001;
    const double rpm_per_mps = 60000.0 / std::abs(options.lead_mm);
    const double braking_speed_mps = std::sqrt(std::max(0.0,
      2.0 * options.acceleration_mps2 * std::max(0.0, std::abs(error) - 0.001)));
    const bool jog_at_limit =
      (jog_direction < 0 && position <= options.min_m + 0.001) ||
      (jog_direction > 0 && position >= options.max_m - 0.001);
    const double desired_speed_mps = jog_direction != 0 ?
      (jog_at_limit ? 0.0 : static_cast<double>(jog_direction) * options.max_speed_mps) :
      (target_reached ? 0.0 :
      std::copysign(std::min(options.max_speed_mps, braking_speed_mps), error));
    const double desired_joint_rpm = desired_speed_mps * rpm_per_mps;
    // The 100 Hz loop limits changes in 60FF so a key press cannot step the drive velocity.
    const double max_rpm_delta = options.acceleration_mps2 * rpm_per_mps * 0.01;
    const double joint_rpm = std::clamp(desired_joint_rpm,
      last_joint_rpm - max_rpm_delta, last_joint_rpm + max_rpm_delta);
    if (!enable) motion_gate = "disabled by operator";
    else if (!backend->pdo_fresh()) motion_gate = "blocked: PDO is not fresh";
    else if (backend->link_state() != EthercatLinkState::operational) motion_gate = "blocked: EtherCAT is not OP";
    else if (backend->working_counter() < 1) motion_gate = "blocked: working counter";
    else if (input.error_code != 0U) motion_gate = "blocked: drive error";
    else if (input.mode_display != 9) motion_gate = "blocked: 6061h is not CSV mode 9";
    else if (state != Cia402State::operation_enabled) motion_gate = "waiting for CiA402 operation enabled";
    else if (jog_direction != 0 && jog_at_limit) motion_gate = "jog blocked: coordinate limit";
    else if (jog_direction != 0) motion_gate = "jog motion permitted";
    else if (target_reached) motion_gate = "target reached";
    else motion_gate = "motion permitted";
    const bool command_motion = decision.allow_motion && enable &&
      (jog_direction != 0 || !target_reached || std::abs(joint_rpm) > 1.0e-6);
    output.target_velocity_units_per_s =
      command_motion ? wire_rpm_to_velocity_units(joint_rpm / (options.sign == 0.0 ? -1.0 : options.sign), units) : 0;
    last_joint_rpm = command_motion ? joint_rpm : 0.0;
    if (!backend->write_pdo(output)) { std::cerr << "PDO 写入失败: " << backend->error_message() << "\n"; break; }
    const auto now = std::chrono::steady_clock::now();
    const auto link_state = backend->link_state();
    const auto wkc = backend->working_counter();
    const bool trace_changed = !have_trace_snapshot ||
      input.status_word != logged_status_word ||
      input.error_code != logged_error_code ||
      input.mode_display != logged_mode ||
      input.digital_inputs != logged_digital_inputs ||
      output.control_word != logged_control_word ||
      wkc != logged_wkc ||
      enable != logged_enable ||
      decision.allow_motion != logged_allow_motion ||
      fault_latched != logged_fault_latched ||
      backend->pdo_fresh() != logged_pdo_fresh ||
      link_state != logged_link ||
      motion_gate != logged_gate;
    if (trace_changed || now >= next_trace) {
      const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - trace_start).count();
      trace << elapsed_ms
            << ",0x" << std::hex << input.status_word
            << ",0x" << (input.status_word & 0x006fU)
            << ",0x" << input.error_code
            << "," << std::dec << static_cast<unsigned int>(input.mode_display)
            << ",0x" << std::hex << input.digital_inputs
            << ",0x" << output.control_word
            << "," << state_name(state)
            << "," << (enable ? 1 : 0)
            << "," << (decision.allow_motion ? 1 : 0)
            << "," << link_state_name(link_state)
            << "," << (backend->pdo_fresh() ? 1 : 0)
            << "," << std::dec << wkc
            << "," << input.actual_position_units
            << "," << input.actual_velocity_units_per_s
            << "," << output.target_velocity_units_per_s
            << "," << std::fixed << std::setprecision(6) << target
            << "," << position
            << ",\"" << motion_gate << "\""
            << "," << (decision.fault ? 1 : 0)
            << "," << (decision.health_ok ? 1 : 0)
            << "," << (decision.mode_ok ? 1 : 0)
            << "," << (decision.recovery_exhausted ? 1 : 0)
            << "," << (fault_latched ? 1 : 0)
            << ",0x" << std::hex << latched_fault_code
            << ",\"" << last_action << "\"\n";
      trace.flush();
      have_trace_snapshot = true;
      logged_status_word = input.status_word;
      logged_error_code = input.error_code;
      logged_mode = input.mode_display;
      logged_digital_inputs = input.digital_inputs;
      logged_control_word = output.control_word;
      logged_wkc = wkc;
      logged_enable = enable;
      logged_allow_motion = decision.allow_motion;
      logged_fault_latched = fault_latched;
      logged_pdo_fresh = backend->pdo_fresh();
      logged_link = link_state;
      logged_gate = motion_gate;
      next_trace = now + std::chrono::milliseconds(100);
    }
    if (!options.command_mode && now >= next_render) {
      std::cout << "\033[2J\033[H" << "Lift EtherCAT test (Master" << options.master << ", " << options.interface << ")\n"
        << "position: " << std::fixed << std::setprecision(4) << position << " m   velocity: " << velocity << " m/s\n"
        << "state: " << state_name(state) << "   status: 0x" << std::hex << std::setw(4) << std::setfill('0') << input.status_word << std::dec << "   mode: " << static_cast<int>(input.mode_display) << "\n"
        << "status mask: 0x" << std::hex << std::setw(4) << (input.status_word & 0x006fU)
        << "   DI(60FD): 0x" << std::setw(8) << input.digital_inputs << std::dec << std::setfill(' ') << "\n"
        << "error: 0x" << std::hex << std::setw(4) << input.error_code << std::dec << " (" << error_name(input.error_code) << ")   WKC: " << backend->working_counter() << "   PDO: " << (backend->pdo_fresh() ? "fresh" : "stale") << "\n"
        << "target: " << target << " m   command: " << (enable ? "ENABLED" : "DISABLED") << "   control: 0x" << std::hex << output.control_word << std::dec << "\n"
        << "profile: " << options.max_speed_mps << " m/s max, " << options.acceleration_mps2 << " m/s2 accel   60FF target: " << output.target_velocity_units_per_s << " units/s\n"
        << "zero: " << zero_offset_status << "\n"
        << "gate: " << motion_gate << "\n"
        << (enable_wait_cycles >= 50U
          ? "diagnostic: 0x000f has been requested for " + std::to_string(enable_wait_cycles * 10U) +
              " ms, but the drive remains Switched On. Check STO, external servo-enable/inhibit inputs, and drive alarm history.\n"
          : "")
        << "last: " << last_action << "\n\n"
        << "[e] enable/hold  [d] disable  [z] zero  [up/down] +/-100 mm task  [hold u/j] jog +/-coordinate  [space] stop  [q] quit\n"
        << "jog release timeout: " << kJogReleaseTimeout.count()
        << " ms; travel: [" << options.min_m << ", " << options.max_m << "] m." << std::endl;
      next_render = now + std::chrono::milliseconds(100);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  output.control_word = 0x0006; output.target_velocity_units_per_s = 0; (void)backend->write_pdo(output); (void)backend->stop();
  if (command_input_flags >= 0) (void)fcntl(STDIN_FILENO, F_SETFL, command_input_flags);
  return 0;
}
