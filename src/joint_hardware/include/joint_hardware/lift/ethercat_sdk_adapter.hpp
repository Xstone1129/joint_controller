#ifndef JOINT_HARDWARE__LIFT__ETHERCAT_SDK_ADAPTER_HPP_
#define JOINT_HARDWARE__LIFT__ETHERCAT_SDK_ADAPTER_HPP_

#include "joint_hardware/lift/ethercat_backend.hpp"

namespace joint_hardware::lift
{

// This is the only file a future board-specific EtherCAT SDK integration should
// replace. It intentionally never reports a link or PDO exchange as successful.
class LiftEthercatSdkAdapter final : public LiftEthercatBackend
{
public:
  bool initialize(const EthercatMasterConfig &) override;
  bool configure_slave(const EthercatSlaveConfig &) override;
  bool read_sdo(uint16_t, uint8_t, std::vector<uint8_t> &) override;
  bool write_sdo(uint16_t, uint8_t, const std::vector<uint8_t> &) override;
  bool start() override;
  bool stop() override;
  bool read_pdo(LiftTxPdo &) override;
  bool write_pdo(const LiftRxPdo &) override;
  bool exchange_pdo(const LiftRxPdo &, LiftTxPdo &) override;
  EthercatLinkState link_state() const noexcept override;
  bool pdo_fresh() const noexcept override;
  uint32_t working_counter() const noexcept override;
  std::string error_message() const override;
};

}  // namespace joint_hardware::lift

#endif  // JOINT_HARDWARE__LIFT__ETHERCAT_SDK_ADAPTER_HPP_
