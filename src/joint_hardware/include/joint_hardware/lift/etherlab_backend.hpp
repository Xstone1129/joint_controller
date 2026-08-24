#ifndef JOINT_HARDWARE__LIFT__ETHERLAB_BACKEND_HPP_
#define JOINT_HARDWARE__LIFT__ETHERLAB_BACKEND_HPP_

#include <memory>

#include "joint_hardware/lift/ethercat_backend.hpp"

namespace joint_hardware::lift
{

// Optional EtherLab/IgH master adapter.  The class is always available at the
// API level, but it returns an explicit "unavailable" error when this package
// was built without the EtherLab development library.
class EtherLabLiftEthercatBackend final : public LiftEthercatBackend
{
public:
  EtherLabLiftEthercatBackend();
  ~EtherLabLiftEthercatBackend() override;

  EtherLabLiftEthercatBackend(const EtherLabLiftEthercatBackend &) = delete;
  EtherLabLiftEthercatBackend & operator=(const EtherLabLiftEthercatBackend &) = delete;

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

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace joint_hardware::lift

#endif  // JOINT_HARDWARE__LIFT__ETHERLAB_BACKEND_HPP_
