#include "joint_hardware/lift/etherlab_backend.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#ifdef JOINT_HARDWARE_HAS_ETHERLAB
#include <ecrt.h>
#endif

namespace joint_hardware::lift
{

class EtherLabLiftEthercatBackend::Impl
{
public:
  const char * error_literal{"EtherLab backend is not initialized"};
  uint32_t last_abort_code{0};
  EthercatLinkState link_state{EthercatLinkState::offline};
  bool pdo_fresh{false};
  uint32_t working_counter{0};

#ifdef JOINT_HARDWARE_HAS_ETHERLAB
  EthercatMasterConfig master_config{};
  EthercatSlaveConfig slave_config{};
  ec_master_t * master{nullptr};
  ec_domain_t * domain{nullptr};
  ec_slave_config_t * slave{nullptr};
  uint8_t * domain_data{nullptr};
  bool initialized{false};
  bool slave_configured{false};
  bool activated{false};
  bool dc_enabled{false};
  std::chrono::steady_clock::time_point cycle_origin{};
  LiftTxPdo input_cache{};
  bool input_cache_valid{false};

  unsigned int rx_control_offset{0};
  unsigned int rx_velocity_offset{0};
  unsigned int rx_torque_offset{0};
  unsigned int tx_status_offset{0};
  unsigned int tx_error_offset{0};
  unsigned int tx_mode_offset{0};
  unsigned int tx_position_offset{0};
  unsigned int tx_velocity_offset{0};
  unsigned int tx_inputs_offset{0};

  void set_error(const char * message) noexcept
  {
    error_literal = message;
    last_abort_code = 0;
  }

  void release_master() noexcept
  {
    if (master != nullptr) {
      if (activated) {
        (void)ecrt_master_deactivate(master);
      }
      ecrt_release_master(master);
    }
    master = nullptr;
    domain = nullptr;
    slave = nullptr;
    domain_data = nullptr;
    initialized = false;
    slave_configured = false;
    activated = false;
    dc_enabled = false;
    cycle_origin = std::chrono::steady_clock::time_point{};
    pdo_fresh = false;
    working_counter = 0;
    link_state = EthercatLinkState::offline;
    input_cache = LiftTxPdo{};
    input_cache_valid = false;
  }

  void refresh_link_state() noexcept
  {
    if (!initialized || master == nullptr || slave == nullptr) {
      link_state = EthercatLinkState::offline;
      return;
    }

    ec_master_state_t master_state{};
    ec_slave_config_state_t slave_state{};
    if (ecrt_master_state(master, &master_state) != 0 ||
      ecrt_slave_config_state(slave, &slave_state) != 0 || !master_state.link_up ||
      !slave_state.online)
    {
      link_state = EthercatLinkState::offline;
      return;
    }

    switch (slave_state.al_state) {
      case EC_AL_STATE_OP:
        link_state = slave_state.operational ? EthercatLinkState::operational :
          EthercatLinkState::safe_operational;
        break;
      case EC_AL_STATE_SAFEOP:
        link_state = EthercatLinkState::safe_operational;
        break;
      case EC_AL_STATE_PREOP:
        link_state = EthercatLinkState::pre_operational;
        break;
      case EC_AL_STATE_INIT:
      default:
        link_state = EthercatLinkState::init;
        break;
    }
  }

  bool register_process_data() noexcept
  {
    static const ec_pdo_entry_info_t rx_entries[] = {
      {0x6040, 0x00, 16},
      {0x60FF, 0x00, 32},
      {0x60B2, 0x00, 16},
    };
    static const ec_pdo_info_t rx_pdos[] = {
      {0x1601, 3, rx_entries},
    };
    static const ec_pdo_entry_info_t tx_entries[] = {
      {0x6041, 0x00, 16},
      {0x603F, 0x00, 16},
      {0x6061, 0x00, 8},
      {0x6064, 0x00, 32},
      {0x606C, 0x00, 32},
      {0x60FD, 0x00, 32},
    };
    static const ec_pdo_info_t tx_pdos[] = {
      {0x1A00, 6, tx_entries},
    };
    static const ec_sync_info_t syncs[] = {
      {2, EC_DIR_OUTPUT, 1, rx_pdos, EC_WD_ENABLE},
      {3, EC_DIR_INPUT, 1, tx_pdos, EC_WD_DISABLE},
      {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DEFAULT},
    };

    if (ecrt_slave_config_pdos(slave, EC_END, syncs) != 0) {
      set_error("EtherLab PDO mapping registration failed");
      return false;
    }

    if (slave_config.dc_assign_activate != 0U) {
      const auto cycle_ns = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          master_config.cycle_period).count());
      if (ecrt_slave_config_dc(
          slave, slave_config.dc_assign_activate, cycle_ns,
          slave_config.dc_sync0_shift_ns, 0, 0) != 0)
      {
        set_error("EtherLab distributed-clock configuration failed");
        return false;
      }
      dc_enabled = true;
    }

    ec_pdo_entry_reg_t registrations[] = {
      {slave_config.alias, slave_config.position, slave_config.vendor_id,
        slave_config.product_code, 0x6040, 0x00, &rx_control_offset, nullptr},
      {slave_config.alias, slave_config.position, slave_config.vendor_id,
        slave_config.product_code, 0x60FF, 0x00, &rx_velocity_offset, nullptr},
      {slave_config.alias, slave_config.position, slave_config.vendor_id,
        slave_config.product_code, 0x60B2, 0x00, &rx_torque_offset, nullptr},
      {slave_config.alias, slave_config.position, slave_config.vendor_id,
        slave_config.product_code, 0x6041, 0x00, &tx_status_offset, nullptr},
      {slave_config.alias, slave_config.position, slave_config.vendor_id,
        slave_config.product_code, 0x603F, 0x00, &tx_error_offset, nullptr},
      {slave_config.alias, slave_config.position, slave_config.vendor_id,
        slave_config.product_code, 0x6061, 0x00, &tx_mode_offset, nullptr},
      {slave_config.alias, slave_config.position, slave_config.vendor_id,
        slave_config.product_code, 0x6064, 0x00, &tx_position_offset, nullptr},
      {slave_config.alias, slave_config.position, slave_config.vendor_id,
        slave_config.product_code, 0x606C, 0x00, &tx_velocity_offset, nullptr},
      {slave_config.alias, slave_config.position, slave_config.vendor_id,
        slave_config.product_code, 0x60FD, 0x00, &tx_inputs_offset, nullptr},
      {}
    };
    if (ecrt_domain_reg_pdo_entry_list(domain, registrations) != 0) {
      set_error("EtherLab PDO entry registration failed");
      return false;
    }
    return true;
  }
#endif
};

EtherLabLiftEthercatBackend::EtherLabLiftEthercatBackend()
: impl_(std::make_unique<Impl>())
{
}

EtherLabLiftEthercatBackend::~EtherLabLiftEthercatBackend()
{
#ifdef JOINT_HARDWARE_HAS_ETHERLAB
  if (impl_ != nullptr) {
    impl_->release_master();
  }
#endif
}

bool EtherLabLiftEthercatBackend::initialize(const EthercatMasterConfig & master)
{
#ifdef JOINT_HARDWARE_HAS_ETHERLAB
  impl_->release_master();
  if (master.cycle_period <= std::chrono::nanoseconds::zero() ||
    master.expected_working_counter == 0U)
  {
    impl_->set_error("EtherLab master cycle and expected working counter must be positive");
    return false;
  }
  const auto cycle_us = std::chrono::duration_cast<std::chrono::microseconds>(
    master.cycle_period);
  if (cycle_us.count() <= 0 ||
    std::chrono::duration_cast<std::chrono::nanoseconds>(cycle_us) != master.cycle_period)
  {
    impl_->set_error("EtherLab master cycle must be an integral number of microseconds");
    return false;
  }

  impl_->master_config = master;
  impl_->master = ecrt_request_master(master.master_index);
  if (impl_->master == nullptr) {
    impl_->set_error(
      "EtherLab master unavailable; check the EtherCAT kernel master and /dev/EtherCAT device");
    return false;
  }
  if (ecrt_master_set_send_interval(
      impl_->master, static_cast<size_t>(cycle_us.count())) != 0)
  {
    impl_->set_error("EtherLab master rejected the configured send interval");
    impl_->release_master();
    return false;
  }
  impl_->domain = ecrt_master_create_domain(impl_->master);
  if (impl_->domain == nullptr) {
    impl_->set_error("EtherLab could not create a process-data domain");
    impl_->release_master();
    return false;
  }
  impl_->initialized = true;
  impl_->link_state = EthercatLinkState::pre_operational;
  impl_->set_error("EtherLab master initialized");
  return true;
#else
  (void)master;
  impl_->error_literal =
    "EtherLab backend unavailable; rebuild with the EtherLab/IgH development library";
  impl_->last_abort_code = 0;
  return false;
#endif
}

bool EtherLabLiftEthercatBackend::configure_slave(const EthercatSlaveConfig & slave)
{
#ifdef JOINT_HARDWARE_HAS_ETHERLAB
  if (impl_->master == nullptr || !impl_->initialized) {
    impl_->set_error("EtherLab master is not initialized");
    return false;
  }
  if (slave.vendor_id == 0U || slave.product_code == 0U) {
    impl_->set_error(
      "EtherLab requires non-zero slave_vendor_id and slave_product_code from ethercat slaves");
    return false;
  }
  if (slave.rx_pdo_bytes != sizeof(LiftRxPdo) || slave.tx_pdo_bytes != sizeof(LiftTxPdo)) {
    impl_->set_error("EtherLab LD3M PDO sizes must be Rx=8 and Tx=17 bytes");
    return false;
  }
  impl_->slave_config = slave;
  impl_->slave = ecrt_master_slave_config(
    impl_->master, slave.alias, slave.position, slave.vendor_id, slave.product_code);
  if (impl_->slave == nullptr) {
    impl_->set_error("EtherLab could not find the configured LD3M slave");
    impl_->slave_configured = false;
    return false;
  }
  impl_->slave_configured = true;
  impl_->link_state = EthercatLinkState::pre_operational;
  impl_->set_error("EtherLab slave configured");
  return true;
#else
  (void)slave;
  impl_->error_literal =
    "EtherLab backend unavailable; rebuild with the EtherLab/IgH development library";
  impl_->last_abort_code = 0;
  return false;
#endif
}

bool EtherLabLiftEthercatBackend::read_sdo(
  uint16_t index, uint8_t subindex, std::vector<uint8_t> & value)
{
#ifdef JOINT_HARDWARE_HAS_ETHERLAB
  if (impl_->master == nullptr || !impl_->slave_configured) {
    impl_->set_error("EtherLab SDO upload requires a configured slave");
    return false;
  }
  std::array<uint8_t, 256> buffer{};
  size_t result_size = 0;
  uint32_t abort_code = 0;
  const int result = ecrt_master_sdo_upload(
    impl_->master, impl_->slave_config.position, index, subindex,
    buffer.data(), buffer.size(), &result_size, &abort_code);
  if (result != 0 || result_size > buffer.size()) {
    impl_->last_abort_code = abort_code;
    impl_->error_literal = "EtherLab SDO upload failed";
    return false;
  }
  value.assign(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(result_size));
  impl_->set_error("EtherLab SDO upload succeeded");
  return true;
#else
  (void)index;
  (void)subindex;
  (void)value;
  impl_->error_literal =
    "EtherLab backend unavailable; rebuild with the EtherLab/IgH development library";
  impl_->last_abort_code = 0;
  return false;
#endif
}

bool EtherLabLiftEthercatBackend::write_sdo(
  uint16_t index, uint8_t subindex, const std::vector<uint8_t> & value)
{
#ifdef JOINT_HARDWARE_HAS_ETHERLAB
  if (impl_->master == nullptr || !impl_->slave_configured) {
    impl_->set_error("EtherLab SDO download requires a configured slave");
    return false;
  }
  if (value.empty()) {
    impl_->set_error("EtherLab refuses an empty SDO download");
    return false;
  }
  uint32_t abort_code = 0;
  const int result = ecrt_master_sdo_download(
    impl_->master, impl_->slave_config.position, index, subindex,
    value.data(), value.size(), &abort_code);
  if (result != 0) {
    impl_->last_abort_code = abort_code;
    impl_->error_literal = "EtherLab SDO download failed";
    return false;
  }
  impl_->set_error("EtherLab SDO download succeeded");
  return true;
#else
  (void)index;
  (void)subindex;
  (void)value;
  impl_->error_literal =
    "EtherLab backend unavailable; rebuild with the EtherLab/IgH development library";
  impl_->last_abort_code = 0;
  return false;
#endif
}

bool EtherLabLiftEthercatBackend::start()
{
#ifdef JOINT_HARDWARE_HAS_ETHERLAB
  if (impl_->master == nullptr || impl_->domain == nullptr || !impl_->slave_configured) {
    impl_->set_error("EtherLab start requires an initialized and configured slave");
    return false;
  }
  if (impl_->activated) {
    return true;
  }
  if (!impl_->register_process_data()) {
    return false;
  }
  if (ecrt_master_activate(impl_->master) != 0) {
    impl_->set_error("EtherLab master activation failed");
    return false;
  }
  impl_->domain_data = ecrt_domain_data(impl_->domain);
  if (impl_->domain_data == nullptr) {
    impl_->set_error("EtherLab domain process-data pointer is null after activation");
    (void)ecrt_master_deactivate(impl_->master);
    return false;
  }
  impl_->activated = true;
  impl_->cycle_origin = std::chrono::steady_clock::now();
  impl_->pdo_fresh = false;
  impl_->working_counter = 0;
  impl_->refresh_link_state();
  impl_->set_error("EtherLab master activated");
  return true;
#else
  impl_->error_literal =
    "EtherLab backend unavailable; rebuild with the EtherLab/IgH development library";
  impl_->last_abort_code = 0;
  return false;
#endif
}

bool EtherLabLiftEthercatBackend::stop()
{
#ifdef JOINT_HARDWARE_HAS_ETHERLAB
  if (impl_->master == nullptr) {
    impl_->link_state = EthercatLinkState::offline;
    impl_->pdo_fresh = false;
    impl_->working_counter = 0;
    return true;
  }
  bool result = true;
  if (impl_->activated && impl_->domain_data != nullptr) {
    EC_WRITE_U16(impl_->domain_data + impl_->rx_control_offset, 0x0006U);
    EC_WRITE_S32(impl_->domain_data + impl_->rx_velocity_offset, 0);
    EC_WRITE_S16(impl_->domain_data + impl_->rx_torque_offset, 0);
    if (ecrt_domain_queue(impl_->domain) != 0 || ecrt_master_send(impl_->master) != 0) {
      result = false;
      impl_->set_error("EtherLab controlled-stop PDO send failed");
    }
    (void)ecrt_master_deactivate(impl_->master);
    impl_->activated = false;
    impl_->domain_data = nullptr;
  }
  impl_->pdo_fresh = false;
  impl_->working_counter = 0;
  impl_->link_state = EthercatLinkState::offline;
  return result;
#else
  impl_->error_literal =
    "EtherLab backend unavailable; rebuild with the EtherLab/IgH development library";
  impl_->last_abort_code = 0;
  return true;
#endif
}

bool EtherLabLiftEthercatBackend::read_pdo(LiftTxPdo & input)
{
#ifdef JOINT_HARDWARE_HAS_ETHERLAB
  input = LiftTxPdo{};
  if (!impl_->activated || impl_->master == nullptr || impl_->domain == nullptr ||
    impl_->domain_data == nullptr)
  {
    impl_->pdo_fresh = false;
    impl_->link_state = EthercatLinkState::offline;
    impl_->set_error("EtherLab PDO read requested before activation");
    return false;
  }

  if (ecrt_master_receive(impl_->master) != 0) {
    impl_->pdo_fresh = false;
    impl_->set_error("EtherLab master receive failed");
    return false;
  }
  if (impl_->dc_enabled) {
    const auto elapsed = std::chrono::steady_clock::now() - impl_->cycle_origin;
    const auto application_time = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count());
    if (ecrt_master_application_time(impl_->master, application_time) != 0 ||
      ecrt_master_sync_reference_clock(impl_->master) != 0 ||
      ecrt_master_sync_slave_clocks(impl_->master) != 0)
    {
      impl_->pdo_fresh = false;
      impl_->set_error("EtherLab distributed-clock synchronization failed");
      return false;
    }
  }
  if (ecrt_domain_process(impl_->domain) != 0) {
    impl_->pdo_fresh = false;
    impl_->set_error("EtherLab domain process failed");
    return false;
  }

  ec_domain_state_t domain_state{};
  if (ecrt_domain_state(impl_->domain, &domain_state) != 0) {
    impl_->pdo_fresh = false;
    impl_->set_error("EtherLab domain state query failed");
    return false;
  }
  impl_->working_counter = domain_state.working_counter;
  impl_->refresh_link_state();

  input.status_word = EC_READ_U16(impl_->domain_data + impl_->tx_status_offset);
  input.error_code = EC_READ_U16(impl_->domain_data + impl_->tx_error_offset);
  input.mode_display = EC_READ_S8(impl_->domain_data + impl_->tx_mode_offset);
  input.actual_position_units = EC_READ_S32(
    impl_->domain_data + impl_->tx_position_offset);
  input.actual_velocity_units_per_s = EC_READ_S32(
    impl_->domain_data + impl_->tx_velocity_offset);
  input.digital_inputs = EC_READ_U32(impl_->domain_data + impl_->tx_inputs_offset);

  const bool working_counter_ok = domain_state.wc_state == EC_WC_COMPLETE &&
    impl_->working_counter >= impl_->master_config.expected_working_counter;
  const bool link_operational = impl_->link_state == EthercatLinkState::operational;
  impl_->pdo_fresh = link_operational && working_counter_ok;

  impl_->input_cache = input;
  impl_->input_cache_valid = true;
  return true;
#else
  (void)input;
  impl_->error_literal =
    "EtherLab backend unavailable; rebuild with the EtherLab/IgH development library";
  impl_->last_abort_code = 0;
  return false;
#endif
}

bool EtherLabLiftEthercatBackend::write_pdo(const LiftRxPdo & output)
{
#ifdef JOINT_HARDWARE_HAS_ETHERLAB
  if (!impl_->activated || impl_->master == nullptr || impl_->domain == nullptr ||
    impl_->domain_data == nullptr)
  {
    impl_->pdo_fresh = false;
    impl_->link_state = EthercatLinkState::offline;
    impl_->set_error("EtherLab PDO write requested before activation");
    return false;
  }

  // The backend applies a second safety gate. A stale/incomplete domain,
  // non-OP slave, drive fault or mode mismatch can never place a non-zero
  // 60FFh value on the wire. Fault-reset control words remain pass-through.
  const LiftTxPdo input = impl_->input_cache_valid ? impl_->input_cache : LiftTxPdo{};
  const bool working_counter_ok = impl_->working_counter >=
    impl_->master_config.expected_working_counter;
  const bool link_operational = impl_->link_state == EthercatLinkState::operational;
  const bool drive_fault = input.error_code != 0U ||
    (input.status_word & 0x006fU) == 0x0008U;
  const bool mode_ok = input.mode_display == 9 || input.mode_display == 6;
  uint16_t control_word = output.control_word;
  const bool fault_reset = (output.control_word & 0x0080U) != 0U;
  if (!link_operational || !working_counter_ok || (!mode_ok && !fault_reset)) {
    control_word = 0x0006U;
  }
  const bool operation_enabled = (input.status_word & 0x006fU) == 0x0027U;
  const int32_t target_velocity =
    (impl_->pdo_fresh && operation_enabled && mode_ok && !drive_fault) ?
    output.target_velocity_units_per_s : 0;

  EC_WRITE_U16(impl_->domain_data + impl_->rx_control_offset, control_word);
  EC_WRITE_S32(impl_->domain_data + impl_->rx_velocity_offset, target_velocity);
  EC_WRITE_S16(impl_->domain_data + impl_->rx_torque_offset, output.torque_feedforward);
  if (ecrt_domain_queue(impl_->domain) != 0 || ecrt_master_send(impl_->master) != 0) {
    impl_->pdo_fresh = false;
    impl_->set_error("EtherLab PDO queue/send failed");
    return false;
  }
  return true;
#else
  (void)output;
  impl_->error_literal =
    "EtherLab backend unavailable; rebuild with the EtherLab/IgH development library";
  impl_->last_abort_code = 0;
  return false;
#endif
}

bool EtherLabLiftEthercatBackend::exchange_pdo(const LiftRxPdo & output, LiftTxPdo & input)
{
  if (!read_pdo(input)) {
    return false;
  }
  return write_pdo(output);
}

EthercatLinkState EtherLabLiftEthercatBackend::link_state() const noexcept
{
  return impl_->link_state;
}

bool EtherLabLiftEthercatBackend::pdo_fresh() const noexcept
{
  return impl_->pdo_fresh;
}

uint32_t EtherLabLiftEthercatBackend::working_counter() const noexcept
{
  return impl_->working_counter;
}

std::string EtherLabLiftEthercatBackend::error_message() const
{
  if (impl_ == nullptr) {
    return "EtherLab backend object is unavailable";
  }
  std::string result = impl_->error_literal;
  if (impl_->last_abort_code != 0U) {
    result += " (SDO abort 0x";
    const char * digits = "0123456789ABCDEF";
    for (int shift = 28; shift >= 0; shift -= 4) {
      result.push_back(digits[(impl_->last_abort_code >> shift) & 0x0FU]);
    }
    result += ")";
  }
  return result;
}

}  // namespace joint_hardware::lift
