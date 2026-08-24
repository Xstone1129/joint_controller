#pragma once

#include <igh_driver_cr.h>
#include <igh_driver_master.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace igh_driver_internal {

constexpr std::size_t kMaxMasterSlaveCount = 24;
constexpr std::size_t kMaxZeroAxisCount = 16;
constexpr std::size_t kAxisStateSlotCount = 23;
constexpr uint8_t kInvalidAxisIndex = 0xFF;
constexpr uint16_t kAliasBase = 0x1000;
constexpr std::size_t kZeroDomainRegCount = 15;
constexpr unsigned int kInvalidPdoOffset = std::numeric_limits<unsigned int>::max();

struct AxisPdoOffsetTable {
    std::array<unsigned int, kMaxZeroAxisCount> operation_mode{};
    std::array<unsigned int, kMaxZeroAxisCount> controlword{};
    std::array<unsigned int, kMaxZeroAxisCount> statusword{};
    std::array<unsigned int, kMaxZeroAxisCount> target_position{};
    std::array<unsigned int, kMaxZeroAxisCount> position_demand{};
    std::array<unsigned int, kMaxZeroAxisCount> actual_position{};
    std::array<unsigned int, kMaxZeroAxisCount> digital_output{};
    std::array<unsigned int, kMaxZeroAxisCount> target_velocity{};
    std::array<unsigned int, kMaxZeroAxisCount> target_torque{};
    std::array<unsigned int, kMaxZeroAxisCount> actual_velocity{};
    std::array<unsigned int, kMaxZeroAxisCount> velocity_offset{};
    std::array<unsigned int, kMaxZeroAxisCount> torque_offset{};
    std::array<unsigned int, kMaxZeroAxisCount> actual_torque{};
    std::array<unsigned int, kMaxZeroAxisCount> actual_encoder_0{};
    std::array<unsigned int, kMaxZeroAxisCount> actual_encoder_1{};
    std::array<unsigned int, kMaxZeroAxisCount> actual_modestate{};
    std::array<unsigned int, kMaxZeroAxisCount> axis_error_code{};

    AxisPdoOffsetTable()
    {
        operation_mode.fill(kInvalidPdoOffset);
        controlword.fill(kInvalidPdoOffset);
        statusword.fill(kInvalidPdoOffset);
        target_position.fill(kInvalidPdoOffset);
        position_demand.fill(kInvalidPdoOffset);
        actual_position.fill(kInvalidPdoOffset);
        digital_output.fill(kInvalidPdoOffset);
        target_velocity.fill(kInvalidPdoOffset);
        target_torque.fill(kInvalidPdoOffset);
        actual_velocity.fill(kInvalidPdoOffset);
        velocity_offset.fill(kInvalidPdoOffset);
        torque_offset.fill(kInvalidPdoOffset);
        actual_torque.fill(kInvalidPdoOffset);
        actual_encoder_0.fill(kInvalidPdoOffset);
        actual_encoder_1.fill(kInvalidPdoOffset);
        actual_modestate.fill(kInvalidPdoOffset);
        axis_error_code.fill(kInvalidPdoOffset);
    }
};

enum class DriveRuntimeProfile : uint8_t {
    None = 0,
    ZeroLegacy,
    ZeroEyou,
    EyouPh,
};

#ifdef SYNC_MASTER_TO_REF
struct DcSyncContext {
    int64_t system_time_base = 0LL;
    uint64_t dc_time_ns = 0ULL;
    int32_t prev_dc_diff_ns = 0;
    int32_t dc_diff_ns = 0;
    unsigned int cycle_ns = PERIOD_NS;
    uint8_t dc_started = 0;
    int64_t dc_diff_total_ns = 0LL;
    int64_t dc_delta_total_ns = 0LL;
    int dc_filter_idx = 0;
    int64_t dc_adjust_ns = 0LL;
    uint64_t dc_start_time_ns = 0ULL;
};
#endif

struct MasterContext {
    int master_index = -1;
    ec_master_t* handle = nullptr;
    uint16_t slave_count = 0;
    std::array<ec_slave_info_t, kMaxMasterSlaveCount> slave_infos{};
    std::array<ec_slave_config_t*, kMaxMasterSlaveCount> slave_configs{};
    std::array<ec_slave_config_state_t, kMaxMasterSlaveCount> slave_states{};
    std::array<bool, kMaxMasterSlaveCount> slave_uses_alias_selection{};
    std::array<uint8_t, kMaxMasterSlaveCount> drive_axis_map{};
    std::array<ec_slave_config_t*, kMaxZeroAxisCount> zero_drive_configs{};
    std::array<ec_sdo_request_t*, kMaxZeroAxisCount> axis_error_code_requests{};
    uint8_t configured_drive_count = 0;
    uint8_t axis_error_poll_cursor = 0;
    uint32_t axis_error_poll_divider = 0;
    ec_domain_t* domain = nullptr;
    uint8_t* process_data = nullptr;
    ec_slave_config_t* reference_drive = nullptr;
#ifdef SYNC_MASTER_TO_REF
    DcSyncContext dc_sync{};
#endif
};

struct CycleCache {
    std::array<int32_t, kAxisStateSlotCount> act_pos{};
    std::array<int32_t, kAxisStateSlotCount> pos_demand{};
    std::array<int32_t, kAxisStateSlotCount> act_vel{};
    std::array<int16_t, kAxisStateSlotCount> act_torque{};
    std::array<int32_t, kAxisStateSlotCount> encoder_0{};
    std::array<int32_t, kAxisStateSlotCount> encoder_1{};
    std::array<uint8_t, kAxisStateSlotCount> mode_word{};
    std::array<uint16_t, kAxisStateSlotCount> status_word{};
    std::array<uint16_t, kAxisStateSlotCount> state{};
    std::array<uint16_t, kAxisStateSlotCount> axis_error_code{};
    bool any_axis_fault = false;
#ifdef MEASURE_PERF
    uint32_t t_cur = 0;
    uint32_t t_prev = 0;
#endif
};

constexpr uint32_t kSharedMotorMonitorDebugMagic = 0x534D4D44U;

struct SharedMotorMonitorDebug {
    uint32_t magic = 0;
    uint8_t monitor_state = 0;
    uint8_t force_shutdown = 0;
    uint8_t armed_latched = 0;
    uint8_t seen_poweroff_after_fault = 0;
    uint8_t ec_poweron = 0;
    uint8_t power_request_allowed = 0;
    uint8_t all_20_enabled = 0;
    uint8_t shared_bad = 0;
    uint8_t shared_bad_axis = kInvalidAxisIndex;
    uint8_t shared_bad_state = 0;
    uint8_t any_axis_fault = 0;
    uint8_t mode_switch_bypass = 0;
    uint32_t update_counter = 0;
};

extern ec_app_read_reg_t ec_shm_desire_axis_data;
extern ec_app_read_reg_t ec_shm_to_axis_data;
extern ec_app_read_reg_t* ec_shm_desire_axis_data_ptr;

extern ec_app_write_reg_t ec_shm_real_axis_data;
extern ec_app_write_reg_t* ec_shm_real_axis_data_ptr;
extern SharedMotorMonitorDebug* shared_motor_monitor_debug_ptr;

extern ec_master_t* master;
extern ec_master_t* master1;

extern std::array<uint8_t, kAxisStateSlotCount> axis_mode_states;
extern std::array<uint8_t, kAxisStateSlotCount> axis_counter_error;
extern std::array<DriveRuntimeProfile, kMaxZeroAxisCount> axis_drive_profiles;

inline bool IsValidAxisIndex(uint8_t axis_index)
{
    return axis_index < kMaxZeroAxisCount;
}

inline bool TryResolveAxisIndex(uint16_t alias, uint8_t& axis_index)
{
    if (alias < kAliasBase) {
        return false;
    }

    const uint16_t resolved_axis = static_cast<uint16_t>(alias - kAliasBase);
    if (resolved_axis >= kMaxZeroAxisCount) {
        return false;
    }

    axis_index = static_cast<uint8_t>(resolved_axis);
    return true;
}

inline bool IsZeroVendorCompatible(uint32_t vendor_id)
{
    return vendor_id == zero_vendor_id || ((vendor_id >> 8) == zero_vendor_id_err);
}

inline bool IsEyouDriveCompatible(uint32_t vendor_id, uint32_t product_code)
{
    return vendor_id == eyou_vendor_id && product_code == eyou_product_code;
}

inline bool IsEyouZeroCompatible(uint32_t vendor_id, uint32_t product_code)
{
    return vendor_id == eyou_vendor_id &&
           (product_code == 0x00221701U ||
            product_code == 0x00222001U ||
            product_code == 0x00222501U);
}

inline bool IsZeroDriveCompatible(uint32_t vendor_id, uint32_t product_code)
{
    return IsZeroVendorCompatible(vendor_id) && product_code == zero_product_code;
}

inline bool IsSupportedZeroLikeDrive(uint32_t vendor_id, uint32_t product_code)
{
    return IsZeroDriveCompatible(vendor_id, product_code) ||
           IsEyouDriveCompatible(vendor_id, product_code) ||
           IsEyouZeroCompatible(vendor_id, product_code);
}

inline DriveRuntimeProfile ResolveDriveRuntimeProfile(uint32_t vendor_id, uint32_t product_code)
{
    if (IsEyouZeroCompatible(vendor_id, product_code)) {
        return DriveRuntimeProfile::ZeroEyou;
    }

    if (IsEyouDriveCompatible(vendor_id, product_code)) {
        return DriveRuntimeProfile::EyouPh;
    }

    if (IsZeroDriveCompatible(vendor_id, product_code)) {
        return DriveRuntimeProfile::ZeroLegacy;
    }

    return DriveRuntimeProfile::None;
}

inline bool IsOptionalOffsetRegistered(unsigned int offset)
{
    return offset != kInvalidPdoOffset;
}

inline const char* GetMasterName(const MasterContext& ctx)
{
    return ctx.master_index == 0 ? "master" : "master1";
}

inline uint16_t getDriveState(uint16_t statusWord)
{
    return statusWord & 0x6f;
}

void timespec_add(struct timespec* result, struct timespec* time1, struct timespec* time2);

#ifdef MEASURE_TIMING
void timespec_sub(struct timespec* result, struct timespec* time1, struct timespec* time2);
#endif

#ifdef SYNC_MASTER_TO_REF
uint64_t system_time_ns(DcSyncContext& dc_sync);
void sync_distributed_clocks(MasterContext& ctx);
void update_master_clock(MasterContext& ctx);
#endif

void ReleaseMasterIfNeeded(ec_master_t*& current_master);
void RegisterSignalHandlers();
void ReleaseAllMastersAndExit(int sig);
bool SetupRealtimeEnvironment();
bool InitializeSharedMemoryRegion();

void axis_mode_state_chage(
    uint8_t modeword,
    uint8_t mode,
    uint8_t axis_num,
    uint16_t statusWord,
    int32_t actpos_tmp,
    int32_t targetpos_tmp,
    int32_t actvel_tmp,
    int32_t targetvel_tmp);

void axis_mode_state_chage_hcfa(
    uint8_t modeword,
    uint8_t mode,
    uint8_t axis_num,
    uint16_t statusWord,
    int32_t actpos_tmp,
    int32_t targetpos_tmp,
    int32_t actvel_tmp,
    int32_t targetvel_tmp);

void axis_ctrword_state_chage(uint8_t ctrstate, uint8_t axis_num);
void hechuan_ctrword_state_chage(uint8_t ctrstate, uint8_t axis_num);
void pxb_ctrword_state_chage(uint8_t ctrstate, uint8_t axis_num);

bool RequestAndProbeMasters(MasterContext& master0, MasterContext& master1_ctx, uint16_t& totalSlaveCount);
void LoadSlaveInfos(MasterContext& ctx);
bool InitializeZeroDrives(MasterContext& ctx);
bool ConfigureZeroPdos(MasterContext& ctx);
std::vector<ec_pdo_entry_reg_t> BuildZeroDomainRegs(const MasterContext& ctx, AxisPdoOffsetTable& offsets);
bool CreateAndRegisterDomain(MasterContext& ctx, const std::vector<ec_pdo_entry_reg_t>& regs);
void ConfigureDistributedClocksIfEnabled(MasterContext& ctx);
bool ActivateMasterAndFetchDomainData(MasterContext& ctx);

void ReceiveAndProcessDomains(MasterContext& ctx, CycleCache& cache);
void ReadAxisFeedbackFromMaster(const MasterContext& ctx, const AxisPdoOffsetTable& offsets, CycleCache& cache);
bool CheckAnyAxisFault(const CycleCache& cache);
void RefreshDesiredCommandsFromSharedMemory();
void UpdateAxisCommandState(uint8_t axis_index, const CycleCache& cache, bool state_is_clean);
void WriteAxisCommandToDomain(uint8_t* process_data, const AxisPdoOffsetTable& offsets, uint8_t axis_index);
void WriteMasterOutputs(const MasterContext& ctx, const AxisPdoOffsetTable& offsets, CycleCache& cache);
void SyncAndSendMaster(MasterContext& ctx);
bool WaitUntilAllSlavesOperational(MasterContext& master0, MasterContext& master1_ctx);
void RunCyclicLoop(MasterContext& master0, MasterContext& master1_ctx, const AxisPdoOffsetTable& offsets);

} // namespace igh_driver_internal
