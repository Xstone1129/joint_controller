#include "joint_hardware/lift/ethercat_sdk_adapter.hpp"

namespace joint_hardware::lift
{

bool LiftEthercatSdkAdapter::initialize(const EthercatMasterConfig &)
{
  return false;
}

bool LiftEthercatSdkAdapter::configure_slave(const EthercatSlaveConfig &)
{
  return false;
}

bool LiftEthercatSdkAdapter::read_sdo(uint16_t, uint8_t, std::vector<uint8_t> &)
{
  return false;
}

bool LiftEthercatSdkAdapter::write_sdo(
  uint16_t, uint8_t, const std::vector<uint8_t> &)
{
  return false;
}

bool LiftEthercatSdkAdapter::start()
{
  return false;
}

bool LiftEthercatSdkAdapter::stop()
{
  return true;
}

bool LiftEthercatSdkAdapter::read_pdo(LiftTxPdo &)
{
  return false;
}

bool LiftEthercatSdkAdapter::write_pdo(const LiftRxPdo &)
{
  return false;
}

bool LiftEthercatSdkAdapter::exchange_pdo(const LiftRxPdo &, LiftTxPdo &)
{
  return false;
}

EthercatLinkState LiftEthercatSdkAdapter::link_state() const noexcept
{
  return EthercatLinkState::offline;
}

bool LiftEthercatSdkAdapter::pdo_fresh() const noexcept
{
  return false;
}

uint32_t LiftEthercatSdkAdapter::working_counter() const noexcept
{
  return 0;
}

std::string LiftEthercatSdkAdapter::error_message() const
{
  return "target EtherCAT SDK adapter unavailable; implement this class for the board master";
}

}  // namespace joint_hardware::lift
