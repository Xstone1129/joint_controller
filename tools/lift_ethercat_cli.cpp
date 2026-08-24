#include "joint_hardware/lift/cia402.hpp"
#include "joint_hardware/lift/ethercat_backend.hpp"
#include "joint_hardware/lift/etherlab_backend.hpp"
#include "joint_hardware/lift/lift_units.hpp"
#include "joint_hardware/lift/pdo_mapping.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <cmath>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <cstdio>
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

std::string error_name(uint16_t code) {
  if (code == 0) return "no error";
  const std::map<uint16_t, std::string> known{{0x2310, "over-current"}, {0x3110, "over-voltage"},
    {0x3210, "under-voltage"}, {0x4210, "over-temperature"}, {0x7300, "encoder fault"}};
  const auto it = known.find(code);
  return it == known.end() ? "vendor-specific/see drive manual" : it->second;
}

bool discover_identity(uint32_t master, uint16_t position, uint32_t & vendor, uint32_t & product) {
  std::ostringstream command;
  command << "ethercat slaves -m " << master << " -v 2>/dev/null";
  FILE * pipe = popen(command.str().c_str(), "r");
  if (pipe == nullptr) return false;
  std::string text; char buffer[512];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) text += buffer;
  const int status = pclose(pipe);
  if (status != 0) return false;
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
  return vendor != 0 && product != 0;
}

struct Options {
  std::string interface{"enp5s0"};
  uint32_t master{2};
  uint16_t position{0};
  uint32_t vendor{0};
  uint32_t product{0};
  double min_m{-1.0};
  double max_m{0.0};
  double lead_mm{10.0};
  double sign{-1.0};
  uint32_t units{10000};
  double max_speed_mps{0.050};
  double acceleration_mps2{0.250};
  bool auto_enable{false};
  bool interface_supplied{false};
  bool master_supplied{false};
  bool position_supplied{false};
  bool vendor_supplied{false};
  bool product_supplied{false};
};

void usage(const char * name) {
  std::cout << "Usage: " << name << " [--enable] [--interface IF] [--master N] [--position N] [--vendor ID] [--product ID] [--speed MPS] [--accel MPS2]\n"
            << "Keys: e enable/hold, d disable, z zero, u +10 mm, j -10 mm, space stop, q quit\n";
}
}  // namespace

int main(int argc, char ** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--enable") options.auto_enable = true;
    else if (arg == "--interface" && i + 1 < argc) { options.interface = argv[++i]; options.interface_supplied = true; }
    else if (arg == "--master" && i + 1 < argc) { options.master = std::stoul(argv[++i], nullptr, 0); options.master_supplied = true; }
    else if (arg == "--position" && i + 1 < argc) { options.position = static_cast<uint16_t>(std::stoul(argv[++i], nullptr, 0)); options.position_supplied = true; }
    else if (arg == "--vendor" && i + 1 < argc) { options.vendor = std::stoul(argv[++i], nullptr, 0); options.vendor_supplied = true; }
    else if (arg == "--product" && i + 1 < argc) { options.product = std::stoul(argv[++i], nullptr, 0); options.product_supplied = true; }
    else if (arg == "--speed" && i + 1 < argc) options.max_speed_mps = std::stod(argv[++i]);
    else if (arg == "--accel" && i + 1 < argc) options.acceleration_mps2 = std::stod(argv[++i]);
    else if (arg == "--help") { usage(argv[0]); return 0; }
  }

  std::signal(SIGINT, stop_signal); std::signal(SIGTERM, stop_signal);
  const auto config = workspace_lift_config();
  if (!options.master_supplied) options.master = config_uint(config, "ethercat_master_index", options.master);
  if (!options.position_supplied) options.position = static_cast<uint16_t>(config_uint(config, "slave_position", options.position));
  if (!options.vendor_supplied) options.vendor = config_uint(config, "slave_vendor_id", options.vendor);
  if (!options.product_supplied) options.product = config_uint(config, "slave_product_code", options.product);
  options.min_m = config_double(config, "position_min_m", options.min_m);
  options.max_m = config_double(config, "position_max_m", options.max_m);
  options.lead_mm = config_double(config, "lead_mm_per_rev", options.lead_mm);
  options.sign = config_double(config, "lift_sign", options.sign);
  options.units = config_uint(config, "command_units_per_rev", options.units);
  if (!std::isfinite(options.max_speed_mps) || options.max_speed_mps <= 0.0 ||
    !std::isfinite(options.acceleration_mps2) || options.acceleration_mps2 <= 0.0) {
    std::cerr << "--speed and --accel must be positive finite values.\n";
    return 2;
  }
  if (!options.interface_supplied) {
    if (const char * env = std::getenv("LIFT_ETHERCAT_INTERFACE")) options.interface = env;
    else if (const auto interface = system_master_interface(options.master); !interface.empty()) options.interface = interface;
  }

  if ((options.vendor == 0 || options.product == 0) && !discover_identity(options.master, options.position, options.vendor, options.product)) {
  std::cerr << "无法自动读取 lift 从站身份。请使用 tools/run_lift_ethercat_cli.sh 启动，"
              << "或确认 /dev/EtherCAT" << options.master << " 存在；也可用 --vendor/--product 覆盖身份。\n";
    return 2;
  }
  std::cout << "发现 lift 从站：vendor=0x" << std::hex << options.vendor
            << " product=0x" << options.product << std::dec << "\n";
  auto backend = make_lift_ethercat_backend("etherlab");
  EthercatMasterConfig master{options.interface, options.master, std::chrono::milliseconds(10), 1};
  EthercatSlaveConfig slave{}; slave.position = options.position; slave.vendor_id = options.vendor; slave.product_code = options.product;
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
  LiftUnitConfig units{}; units.lead_mm_per_rev = options.lead_mm; units.lift_sign = options.sign;
  units.command_units_per_rev = options.units; units.effective_command_units_per_rev = options.units;

  TerminalRawMode terminal;
  if (!terminal.active()) {
    std::cerr << "CLI 必须在交互终端中运行。\n";
    (void)backend->stop();
    return 2;
  }
  LiftRxPdo output{}; LiftTxPdo input{}; Cia402Controller cia(3, 10);
  bool enable = false; double target = 0.0; int32_t zero = 0; bool have_sample = false;
  double last_joint_rpm = 0.0;
  std::string last_action{"waiting for first PDO"};
  std::string motion_gate{"waiting for first PDO"};
  constexpr double kStepM = 0.010;
  auto issue_key = [&](char key) {
    const double position = position_units_to_m(input.actual_position_units, zero, units);
    if (!have_sample && key != 'q' && key != 'Q') return;
    switch (key) {
      case 'e': case 'E': enable = true; target = position; last_action = "enabled: holding current position"; break;
      case 'd': case 'D': enable = false; target = position; last_action = "disabled"; break;
      case 'z': case 'Z': zero = input.actual_position_units; target = 0.0; last_action = "current position set to zero"; break;
      case 'u': case 'U': {
        const double requested = target + kStepM;
        target = std::clamp(requested, options.min_m, options.max_m);
        enable = true;
        last_action = target == requested ? "target +10 mm" : "target is already at + coordinate limit";
        break;
      }
      case 'j': case 'J': {
        const double requested = target - kStepM;
        target = std::clamp(requested, options.min_m, options.max_m);
        enable = true;
        last_action = target == requested ? "target -10 mm" : "target is already at - coordinate limit";
        break;
      }
      case ' ': enable = false; target = position; last_action = "stopped and disabled"; break;
      case 'q': case 'Q': running.store(false); break;
      default: break;
    }
  };
  std::cout << "lift EtherCAT CLI 已连接。" << std::endl;
  if (options.auto_enable) { enable = true; last_action = "enable requested"; }
  auto next_render = std::chrono::steady_clock::now();
  while (running.load()) {
    char keys[32];
    const ssize_t count = read(STDIN_FILENO, keys, sizeof(keys));
    if (count > 0) {
      for (ssize_t index = 0; index < count; ++index) issue_key(keys[index]);
    } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      last_action = "terminal read error";
      running.store(false);
    }
    if (!backend->read_pdo(input)) { std::cerr << "PDO 读取失败: " << backend->error_message() << "\n"; break; }
    have_sample = true;
    const auto state = parse_cia402_status(input.status_word);
    const bool healthy = backend->pdo_fresh() && backend->link_state() == EthercatLinkState::operational;
    Cia402Inputs cia_in{}; cia_in.status_word = input.status_word; cia_in.error_code = input.error_code; cia_in.mode_display = input.mode_display;
    cia_in.link_operational = healthy; cia_in.pdo_fresh = healthy; cia_in.working_counter_ok = backend->working_counter() >= 1; cia_in.request_enable = enable; cia_in.request_fault_reset = true;
    const auto decision = cia.step(cia_in);
    output.control_word = decision.control_word;
    const double position = position_units_to_m(input.actual_position_units, zero, units);
    if (last_action == "waiting for first PDO") { target = position; last_action = "disabled; holding current position"; }
    const double error = target - position;
    const double velocity = velocity_units_to_mps(input.actual_velocity_units_per_s, units);
    const bool target_reached = std::abs(error) <= 0.001;
    const double rpm_per_mps = 60000.0 / std::abs(options.lead_mm);
    const double braking_speed_mps = std::sqrt(std::max(0.0,
      2.0 * options.acceleration_mps2 * std::max(0.0, std::abs(error) - 0.001)));
    const double desired_speed_mps = target_reached ? 0.0 :
      std::copysign(std::min(options.max_speed_mps, braking_speed_mps), error);
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
    else if (target_reached) motion_gate = "target reached";
    else motion_gate = "motion permitted";
    const bool command_motion = decision.allow_motion && enable && !target_reached;
    output.target_velocity_units_per_s =
      command_motion ? wire_rpm_to_velocity_units(joint_rpm / (options.sign == 0.0 ? -1.0 : options.sign), units) : 0;
    last_joint_rpm = command_motion ? joint_rpm : 0.0;
    if (!backend->write_pdo(output)) { std::cerr << "PDO 写入失败: " << backend->error_message() << "\n"; break; }
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_render) {
      std::cout << "\033[2J\033[H" << "Lift EtherCAT test (Master" << options.master << ", " << options.interface << ")\n"
        << "position: " << std::fixed << std::setprecision(4) << position << " m   velocity: " << velocity << " m/s\n"
        << "state: " << state_name(state) << "   status: 0x" << std::hex << std::setw(4) << std::setfill('0') << input.status_word << std::dec << "   mode: " << static_cast<int>(input.mode_display) << "\n"
        << "error: 0x" << std::hex << std::setw(4) << input.error_code << std::dec << " (" << error_name(input.error_code) << ")   WKC: " << backend->working_counter() << "   PDO: " << (backend->pdo_fresh() ? "fresh" : "stale") << "\n"
        << "target: " << target << " m   command: " << (enable ? "ENABLED" : "DISABLED") << "   control: 0x" << std::hex << output.control_word << std::dec << "\n"
        << "profile: " << options.max_speed_mps << " m/s max, " << options.acceleration_mps2 << " m/s2 accel   60FF target: " << output.target_velocity_units_per_s << " units/s\n"
        << "gate: " << motion_gate << "\n"
        << "last: " << last_action << "\n\n"
        << "[e] enable/hold  [d] disable  [z] zero  [u] +coordinate 10mm  [j] -coordinate 10mm  [space] stop  [q] quit\n"
        << "travel: [" << options.min_m << ", " << options.max_m << "] m; after z at 0, use [j] to enter the configured range." << std::endl;
      next_render = now + std::chrono::milliseconds(100);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  output.control_word = 0x0006; output.target_velocity_units_per_s = 0; (void)backend->write_pdo(output); (void)backend->stop();
  return 0;
}
