#ifndef JOINT_HARDWARE__LIFT__ETHERCAT_BACKEND_HPP_
#define JOINT_HARDWARE__LIFT__ETHERCAT_BACKEND_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace joint_hardware::lift
{

// The backend owns all SDK-specific details.  LiftHardware only exchanges these
// fixed-width, little-endian PDO layouts and performs SDO work outside its cycle.
enum class EthercatLinkState : uint8_t
{
  offline,
  init,
  pre_operational,
  safe_operational,
  operational
};

struct EthercatMasterConfig
{
  std::string interface_name;
  uint32_t master_index{0};
  std::chrono::nanoseconds cycle_period{std::chrono::milliseconds(10)};
  uint32_t expected_working_counter{1};
};

struct EthercatSlaveConfig
{
  uint16_t alias{0};
  uint16_t position{0};
  uint32_t vendor_id{0};
  uint32_t product_code{0};
  // These are assigned by the EtherCAT master/FMMU configuration, not by the
  // LD3M manual. Zero means that the future raw-socket adapter must discover
  // them while configuring the slave.
  uint32_t rx_pdo_logical_address{0};
  uint32_t tx_pdo_logical_address{0};
  uint16_t rx_pdo_bytes{8};
  uint16_t tx_pdo_bytes{17};
  // EtherLab needs the vendor-specific AssignActivate word from the drive's
  // ESI file to enable distributed clocks. Zero deliberately means "do not
  // guess" and leaves DC disabled until the value is confirmed.
  uint16_t dc_assign_activate{0};
  int32_t dc_sync0_shift_ns{0};
};

#pragma pack(push, 1)
struct LiftRxPdo
{
  uint16_t control_word{0};
  int32_t target_velocity_units_per_s{0};
  int16_t torque_feedforward{0};
};

struct LiftTxPdo
{
  uint16_t status_word{0};
  uint16_t error_code{0};
  int8_t mode_display{0};
  int32_t actual_position_units{0};
  int32_t actual_velocity_units_per_s{0};
  uint32_t digital_inputs{0};
};
#pragma pack(pop)

static_assert(sizeof(LiftRxPdo) == 8, "RxPDO layout must be 8 bytes");
static_assert(offsetof(LiftRxPdo, control_word) == 0, "6040h must be first in RxPDO");
static_assert(
  offsetof(LiftRxPdo, target_velocity_units_per_s) == 2,
  "60FFh must follow 6040h");
static_assert(
  offsetof(LiftRxPdo, torque_feedforward) == 6,
  "60B2h must follow 60FFh");
static_assert(sizeof(LiftTxPdo) == 17, "TxPDO layout must be 17 bytes");
static_assert(offsetof(LiftTxPdo, status_word) == 0, "6041h must be first in TxPDO");
static_assert(offsetof(LiftTxPdo, error_code) == 2, "603Fh must follow 6041h");
static_assert(offsetof(LiftTxPdo, mode_display) == 4, "6061h must follow 603Fh");
static_assert(
  offsetof(LiftTxPdo, actual_position_units) == 5,
  "6064h must follow 6061h");
static_assert(
  offsetof(LiftTxPdo, actual_velocity_units_per_s) == 9,
  "606Ch must follow 6064h");
static_assert(
  offsetof(LiftTxPdo, digital_inputs) == 13,
  "60FDh must follow 606Ch");

using LiftRxPdoWire = std::array<uint8_t, sizeof(LiftRxPdo)>;
using LiftTxPdoWire = std::array<uint8_t, sizeof(LiftTxPdo)>;

// EtherCAT PDOs are canonical little-endian byte streams. SDK adapters should
// use these helpers instead of relying on host struct endianness when copying
// process data to or from a master-specific buffer.
bool encode_lift_rx_pdo_le(const LiftRxPdo & pdo, LiftRxPdoWire & wire) noexcept;
bool decode_lift_rx_pdo_le(const LiftRxPdoWire & wire, LiftRxPdo & pdo) noexcept;
bool encode_lift_tx_pdo_le(const LiftTxPdo & pdo, LiftTxPdoWire & wire) noexcept;
bool decode_lift_tx_pdo_le(const LiftTxPdoWire & wire, LiftTxPdo & pdo) noexcept;

class LiftEthercatBackend
{
public:
  virtual ~LiftEthercatBackend() = default;

  virtual bool initialize(const EthercatMasterConfig & master) = 0;
  virtual bool configure_slave(const EthercatSlaveConfig & slave) = 0;
  virtual bool read_sdo(
    uint16_t index, uint8_t subindex, std::vector<uint8_t> & value) = 0;
  virtual bool write_sdo(
    uint16_t index, uint8_t subindex, const std::vector<uint8_t> & value) = 0;
  virtual bool start() = 0;
  virtual bool stop() = 0;

  // The 100 Hz path uses these two operations separately: read_pdo() consumes
  // the frame received for the current cycle, while write_pdo() queues and
  // sends the command calculated by ros2_control in that same cycle.  Neither
  // operation may allocate, wait for an SDO response, or retry without a
  // bounded result.
  virtual bool read_pdo(LiftTxPdo & input) = 0;
  virtual bool write_pdo(const LiftRxPdo & output) = 0;

  // Compatibility helper for callers that still want one combined exchange.
  // Concrete backends retain this method because a mock or a legacy caller may
  // expect the returned sample to reflect the supplied output immediately.
  virtual bool exchange_pdo(const LiftRxPdo & output, LiftTxPdo & input) = 0;

  virtual EthercatLinkState link_state() const noexcept = 0;
  virtual bool pdo_fresh() const noexcept = 0;
  virtual uint32_t working_counter() const noexcept = 0;
  virtual std::string error_message() const = 0;
};

class MockLiftEthercatBackend final : public LiftEthercatBackend
{
public:
  MockLiftEthercatBackend();
  ~MockLiftEthercatBackend() override = default;

  bool initialize(const EthercatMasterConfig & master) override;
  bool configure_slave(const EthercatSlaveConfig & slave) override;
  bool read_sdo(uint16_t index, uint8_t subindex, std::vector<uint8_t> & value) override;
  bool write_sdo(
    uint16_t index, uint8_t subindex, const std::vector<uint8_t> & value) override;
  bool start() override;
  bool stop() override;
  bool read_pdo(LiftTxPdo & input) override;
  bool write_pdo(const LiftRxPdo & output) override;
  bool exchange_pdo(const LiftRxPdo & output, LiftTxPdo & input) override;
  EthercatLinkState link_state() const noexcept override;
  bool pdo_fresh() const noexcept override;
  uint32_t working_counter() const noexcept override;
  std::string error_message() const override;

  // Fault/dropout controls make the backend useful for deterministic offline tests.
  void set_online(bool online);
  void set_pdo_fresh(bool fresh);
  void set_working_counter(uint32_t working_counter);
  void inject_fault(uint16_t error_code);
  void set_mode(int8_t mode);
  void set_actual_position_units(int32_t position_units);
  void set_startup_position_sequence(std::vector<int32_t> position_units);
  void set_digital_inputs(uint32_t digital_inputs);
  void set_quick_stop_velocity_units(int32_t velocity_units_per_s);
  void set_brake_p06_14_ms(uint32_t stop_time_ms);
  const LiftRxPdo & last_output() const noexcept;
  const LiftTxPdo & current_input() const noexcept;

private:
  EthercatMasterConfig master_{};
  EthercatSlaveConfig slave_{};
  EthercatLinkState link_state_{EthercatLinkState::offline};
  LiftRxPdo last_output_{};
  LiftTxPdo input_{};
  bool initialized_{false};
  bool configured_{false};
  bool pdo_fresh_{false};
  uint32_t working_counter_{0};
  std::vector<int32_t> startup_position_sequence_;
  std::size_t startup_position_index_{0};
  uint32_t encoder_counts_per_rev_{131072};
  uint32_t gear_ratio_numerator_{1};
  uint32_t gear_ratio_denominator_{1};
  uint32_t feed_units_numerator_{10000};
  uint32_t p00_08_command_units_per_rev_{0};
  int8_t homing_method_{19};
  uint32_t homing_speed_high_units_s_{10000};
  uint32_t homing_speed_low_units_s_{5000};
  uint32_t homing_acceleration_units_s2_{500000};
  int32_t homing_offset_units_{0};
  uint16_t absolute_encoder_setting_{0};
  uint32_t brake_p04_37_ms_{150};
  uint32_t brake_p04_38_ms_{0};
  uint32_t brake_p04_39_rpm_{30};
  uint32_t brake_p06_14_ms_{500};
  uint32_t brake_p05_06_mode_{3};
  uint32_t brake_p05_10_mode_{4};
  int32_t quick_stop_velocity_units_per_s_{0};
  int8_t mode_{9};
  std::string error_message_{};
};

// Placeholder used when a real SDK adapter has not been supplied.  It is
// deliberately unavailable and never reports a successful connection.
class UnavailableLiftEthercatBackend final : public LiftEthercatBackend
{
public:
  bool initialize(const EthercatMasterConfig &) override;
  bool configure_slave(const EthercatSlaveConfig &) override;
  bool read_sdo(uint16_t, uint8_t, std::vector<uint8_t> &) override;
  bool write_sdo(uint16_t, uint8_t, const std::vector<uint8_t> &) override;
  bool start() override;
  bool stop() override;
  bool read_pdo(LiftTxPdo & input) override;
  bool write_pdo(const LiftRxPdo & output) override;
  bool exchange_pdo(const LiftRxPdo &, LiftTxPdo &) override;
  EthercatLinkState link_state() const noexcept override;
  bool pdo_fresh() const noexcept override;
  uint32_t working_counter() const noexcept override;
  std::string error_message() const override;
};

std::unique_ptr<LiftEthercatBackend> make_lift_ethercat_backend(const std::string & name);

}  // namespace joint_hardware::lift

#endif  // JOINT_HARDWARE__LIFT__ETHERCAT_BACKEND_HPP_
