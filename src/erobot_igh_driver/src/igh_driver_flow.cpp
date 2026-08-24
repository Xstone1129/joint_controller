#include "igh_driver_internal.h"

#include <algorithm>
#include <cstring>

namespace igh_driver_internal {

namespace {

/* Legacy Zero PDO: 6060/6061 are 16-bit in the working map. */
ec_pdo_entry_info_t zero_legacy_pdo_entries[] = {
    {0x6040, 0x00, 16},
    {0x607A, 0x00, 32},
    {0x60FF, 0x00, 32},
    {0x6071, 0x00, 16},
    {0x6060, 0x00, 16},
    {0x6041, 0x00, 16},
    {0x6064, 0x00, 32},
    {0x606C, 0x00, 32},
    {0x6077, 0x00, 16},
    {0x6061, 0x00, 16},
};

ec_pdo_info_t zero_legacy_pdos[] = {
    {0x1600, 5, zero_legacy_pdo_entries + 0},
    {0x1A00, 5, zero_legacy_pdo_entries + 5},
};

ec_sync_info_t zero_legacy_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, zero_legacy_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, zero_legacy_pdos + 1, EC_WD_DISABLE},
    {0xFF},
};

/* EYOU zero-compatible PDO: keep 60FE and the 202A/202B encoder objects. */
ec_pdo_entry_info_t zero_eyou_pdo_entries[] = {
    {0x6040, 0x00, 16},
    {0x607A, 0x00, 32},
    {0x60FE, 0x00, 32},
    {0x6071, 0x00, 16},
    {0x6060, 0x00, 8},
    {0x0000, 0x00, 8},
    {0x6041, 0x00, 16},
    {0x6064, 0x00, 32},
    {0x606C, 0x00, 32},
    {0x6077, 0x00, 16},
    {0x202A, 0x00, 32},
    {0x202B, 0x00, 32},
    {0x6061, 0x00, 8},
    {0x0000, 0x00, 8},
};

ec_pdo_info_t zero_eyou_pdos[] = {
    {0x1600, 6, zero_eyou_pdo_entries + 0},
    {0x1A00, 8, zero_eyou_pdo_entries + 6},
};

ec_sync_info_t zero_eyou_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, zero_eyou_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, zero_eyou_pdos + 1, EC_WD_DISABLE},
    {0xFF},
};

/* EYOU PH PDO: keep 16-bit alignment and the working PH object order. */
ec_pdo_entry_info_t eyou_ph_pdo_entries[] = {
    {0x6040, 0x00, 16},
    {0x607A, 0x00, 32},
    {0x60FF, 0x00, 32},
    {0x6071, 0x00, 16},
    {0x6060, 0x00, 8},
    {0x0000, 0x00, 8},
    {0x6041, 0x00, 16},
    {0x6064, 0x00, 32},
    {0x606C, 0x00, 32},
    {0x6077, 0x00, 16},
    {0x6061, 0x00, 8},
    {0x603F, 0x00, 16},
    {0x0000, 0x00, 8},
};

ec_pdo_info_t eyou_ph_pdos[] = {
    {0x1600, 6, eyou_ph_pdo_entries + 0},
    {0x1A00, 7, eyou_ph_pdo_entries + 6},
};

ec_sync_info_t eyou_ph_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, eyou_ph_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, eyou_ph_pdos + 1, EC_WD_DISABLE},
    {0xFF},
};

constexpr uint8_t kModeSwitchAxisCount = 14;
constexpr uint8_t kFixedCspAxisStart = kModeSwitchAxisCount;
constexpr uint8_t kFixedCspAxisEnd = kMaxZeroAxisCount;
constexpr uint8_t kModeSwitchLogAxisFirst = 0;
constexpr uint8_t kModeSwitchLogAxisMiddle = 7;
constexpr uint8_t kModeSwitchLogAxisLast = kModeSwitchAxisCount - 1;
constexpr int32_t kStandstillVelocityThreshold = 100;
constexpr uint32_t kStandstillRequiredCycles = FREQUENCY / 5;
constexpr uint32_t kModeDwellCycles = FREQUENCY * 10;
constexpr uint32_t kAxisErrorLogPeriodCycles = FREQUENCY / 10;
constexpr uint32_t kAxisDiagnosticLogPeriodCycles = FREQUENCY;
constexpr uint32_t kAxisErrorPollPeriodCycles = 4;
constexpr uint32_t kAxisErrorRequestTimeoutMs = 50;
constexpr uint32_t kCspReentryTakeoverStableCycles = FREQUENCY / 50;
constexpr uint32_t kEyouCstToCstDisableHoldCycles = FREQUENCY / 5;
constexpr uint32_t kEyouCstToCstModeHoldCycles = FREQUENCY / 5;
constexpr uint32_t kEyouCstToCspDisableHoldCycles = FREQUENCY / 20;
constexpr uint32_t kEyouCstToCspModeHoldCycles = FREQUENCY / 5;
constexpr uint16_t kArmAxisFeedforwardGain = 0x0000;
constexpr uint16_t kTailAxisFeedforwardGain = 0x4000;
constexpr uint32_t kArmAxisProtectionWindow = 300000U;
// The current workspace owns the 14 arm drives only. The factory stack's
// additional shared slots are not populated by this arm-only deployment.
constexpr std::size_t kSharedMotorStatusAxisCount = kModeSwitchAxisCount;
constexpr uint32_t kEyouZeroPh17ProductCode = 0x00221701U;
constexpr uint32_t kEyouZeroPh20ProductCode = 0x00222001U;
constexpr uint32_t kEyouZeroPh25ProductCode = 0x00222501U;

struct DriveProfile;
using DriveProfileInitFn = void (*)(const DriveProfile&, ec_master_t*, uint16_t, uint8_t);

struct DriveProfile {
    const char* name;
    DriveRuntimeProfile runtime_profile;
    uint32_t vendor_id;
    uint32_t product_code;
    ec_sync_info_t* syncs;
    std::size_t domain_reg_count;
    uint16_t dc_assign_activate;
    DriveProfileInitFn init;
    bool supports_cst;
    bool has_zero_objects;
    uint16_t encoder_0_index;
    uint16_t encoder_1_index;
};

constexpr std::size_t kZeroLegacyDomainRegCount = 10;
constexpr std::size_t kZeroEyouDomainRegCount = 12;
constexpr std::size_t kEyouPhDomainRegCount = 11;
constexpr uint32_t kCstTakeoverBlendCycles =
    (FREQUENCY / 50) > 0 ? (FREQUENCY / 50) : 1;

void InitZeroDriveProfile(
    const DriveProfile& profile,
    ec_master_t* master_handle,
    uint16_t slave_pos,
    uint8_t axis_index)
{
    const uint16_t feedforward_gain =
        axis_index < kModeSwitchAxisCount ? kArmAxisFeedforwardGain : kTailAxisFeedforwardGain;
    if (profile.has_zero_objects) {
        initDrive_zero(master_handle, slave_pos, CONTROL_WORD_CSP, feedforward_gain);
    } else {
        initDrive_eyou_basic(master_handle, slave_pos, CONTROL_WORD_CSP);
    }
    if (profile.has_zero_objects && axis_index < kModeSwitchAxisCount) {
        ODwrite_u32(master_handle, slave_pos, 0x3B60, 0x00, kArmAxisProtectionWindow);
        ODwrite_u32(master_handle, slave_pos, 0x3B61, 0x00, kArmAxisProtectionWindow);
        ODwrite_u32(master_handle, slave_pos, 0x3B62, 0x00, kArmAxisProtectionWindow);
    }
}

void InitEyouDriveProfile(
    const DriveProfile& profile,
    ec_master_t* master_handle,
    uint16_t slave_pos,
    uint8_t axis_index)
{
    (void)profile;
    (void)axis_index;
    initDrive_eyou_basic(master_handle, slave_pos, CONTROL_WORD_CSP);
}

const DriveProfile drive_profiles[] = {
    {
        "ZeroLegacy",
        DriveRuntimeProfile::ZeroLegacy,
        zero_vendor_id,
        zero_product_code,
        zero_legacy_syncs,
        kZeroLegacyDomainRegCount,
        0x0300,
        InitZeroDriveProfile,
        true,
        true,
        0,
        0,
    },
    {
        "ZeroEyou",
        DriveRuntimeProfile::ZeroEyou,
        eyou_vendor_id,
        0,
        zero_eyou_syncs,
        kZeroEyouDomainRegCount,
        0x0300,
        InitZeroDriveProfile,
        true,
        false,
        0x202A,
        0x202B,
    },
    {
        "EyouPh",
        DriveRuntimeProfile::EyouPh,
        eyou_vendor_id,
        eyou_product_code,
        eyou_ph_syncs,
        kEyouPhDomainRegCount,
        0x0300,
        InitEyouDriveProfile,
        true,
        false,
        0,
        0,
    },
};

enum class ModeSwitchTestState : uint8_t {
    BootHoldCsp,
    EnableAxis,
    WaitStandstillInCsp,
    CspDwell,
    RequestCst,
    WaitCstAck,
    CstDwell,
    RequestCsp,
    WaitCspAck,
    FaultHold,
};

enum class CspReentryEnableState : uint8_t {
    Idle,
    DropEnableForCsp,
    WaitDisabledForCsp,
    WaitModeAckInCsp,
    EnableCsp,
    WaitCspReady,
};

enum class EyouCstEnableState : uint8_t {
    Idle,
    Shutdown,
    SwitchOn,
    EnableOperation,
};

enum class EyouCstToCspTransitionState : uint8_t {
    Idle,
    DisableHold,
    ModeHold,
    ReEnable,
};

enum class EyouCstToCstTransitionState : uint8_t {
    Idle,
    DisableHold,
    ModeHold,
    ReEnable,
};

enum class SharedMotorMonitorState : uint8_t {
    PowerOff,
    Enabling,
    Armed,
    Faulted,
};

struct ModeSwitchTestContext {
    ModeSwitchTestState state = ModeSwitchTestState::BootHoldCsp;
    uint8_t requested_mode = AXIS_MODE_CSP;
    int32_t hold_position = 0;
    uint32_t standstill_cycles = 0;
    uint32_t dwell_cycles = 0;
    bool wait_standstill_announced = false;
};

ModeSwitchTestContext mode_switch_test_ctx{};
std::array<int32_t, kMaxZeroAxisCount> hold_positions{};
std::array<bool, kMaxZeroAxisCount> hold_position_initialized{};
std::array<bool, kMaxZeroAxisCount> mode_switch_axis_configured{};
std::array<bool, kMaxZeroAxisCount> cst_takeover_active{};
std::array<int16_t, kMaxZeroAxisCount> cst_takeover_start_torque{};
std::array<uint32_t, kMaxZeroAxisCount> cst_takeover_cycles{};
bool arm_group_prepared_this_cycle = false;
uint8_t arm_group_requested_mode = AXIS_MODE_CSP;
uint8_t arm_group_previous_mode = AXIS_MODE_CSP;
bool arm_group_power_was_requested = false;
bool arm_group_hold_active = false;
EyouCstEnableState eyou_cst_enable_state = EyouCstEnableState::Idle;
EyouCstToCstTransitionState eyou_cst_to_cst_state = EyouCstToCstTransitionState::Idle;
uint32_t eyou_cst_to_cst_cycles = 0;
EyouCstToCspTransitionState eyou_cst_to_csp_state = EyouCstToCspTransitionState::Idle;
uint32_t eyou_cst_to_csp_cycles = 0;
bool csp_reentry_hold_active = false;
bool csp_reentry_enable_pending = false;
CspReentryEnableState csp_reentry_enable_state = CspReentryEnableState::Idle;
uint32_t csp_reentry_ready_cycles = 0;
uint32_t csp_reentry_takeover_cycles = 0;
bool force_shutdown_all_zero_axes = false;
bool shared_motor_monitor_previous_poweron = false;
bool shared_motor_monitor_armed_latched = false;
bool shared_motor_monitor_seen_poweroff_after_fault = true;
SharedMotorMonitorState shared_motor_monitor_state = SharedMotorMonitorState::PowerOff;

inline int32_t AbsI32(int32_t value)
{
    return value >= 0 ? value : -value;
}

bool IsAxisFaulted(const CycleCache& cache, uint8_t axis_index)
{
    if (!IsValidAxisIndex(axis_index)) {
        return false;
    }

    if ((cache.status_word[axis_index] & AXIS_STATUES_ERR) != 0 ||
        cache.axis_error_code[axis_index] != 0) {
        return true;
    }

    return ec_shm_real_axis_data_ptr != nullptr &&
           (ec_shm_real_axis_data_ptr->axis_state[axis_index].ec_ctrstate & AXIS_STATUES_ERR) != 0;
}

bool AreAllModeSwitchAxesOperationEnabled(const CycleCache& cache);

bool IsModeSwitchAxis(uint8_t axis_index)
{
    return axis_index < kModeSwitchAxisCount;
}

bool IsFixedCspAxis(uint8_t axis_index)
{
    return axis_index >= kFixedCspAxisStart && axis_index < kFixedCspAxisEnd;
}

const DriveProfile* FindDriveProfile(uint32_t vendor_id, uint32_t product_code)
{
    for (const DriveProfile& profile : drive_profiles) {
        bool identity_matches = false;
        switch (profile.runtime_profile) {
        case DriveRuntimeProfile::ZeroLegacy:
            identity_matches = IsZeroVendorCompatible(vendor_id) && product_code == zero_product_code;
            break;
        case DriveRuntimeProfile::ZeroEyou:
            identity_matches =
                vendor_id == eyou_vendor_id &&
                (product_code == kEyouZeroPh17ProductCode ||
                 product_code == kEyouZeroPh20ProductCode ||
                 product_code == kEyouZeroPh25ProductCode);
            break;
        case DriveRuntimeProfile::EyouPh:
            identity_matches = vendor_id == profile.vendor_id && product_code == profile.product_code;
            break;
        case DriveRuntimeProfile::None:
            break;
        }

        if (identity_matches) {
            return &profile;
        }
    }

    return nullptr;
}

const DriveProfile* GetAxisDriveProfile(uint8_t axis_index)
{
    if (!IsValidAxisIndex(axis_index)) {
        return nullptr;
    }

    const DriveRuntimeProfile runtime_profile = axis_drive_profiles[axis_index];
    for (const DriveProfile& profile : drive_profiles) {
        if (profile.runtime_profile == runtime_profile) {
            return &profile;
        }
    }

    return nullptr;
}

bool IsAxisConfigured(uint8_t axis_index)
{
    return IsValidAxisIndex(axis_index) &&
           axis_drive_profiles[axis_index] != DriveRuntimeProfile::None;
}

bool IsZeroLikeProfile(DriveRuntimeProfile runtime_profile)
{
    return runtime_profile == DriveRuntimeProfile::ZeroLegacy ||
           runtime_profile == DriveRuntimeProfile::ZeroEyou;
}

bool IsCstModeSwitchAxis(uint8_t axis_index)
{
    if (axis_index >= kModeSwitchAxisCount || !mode_switch_axis_configured[axis_index]) {
        return false;
    }

    const DriveProfile* drive_profile = GetAxisDriveProfile(axis_index);
    return drive_profile != nullptr && drive_profile->supports_cst;
}

bool IsEyouPhModeSwitchAxis(uint8_t axis_index)
{
    return IsCstModeSwitchAxis(axis_index) &&
           axis_drive_profiles[axis_index] == DriveRuntimeProfile::EyouPh;
}

bool HasAnyEyouPhModeSwitchAxis()
{
    for (uint8_t axis_index = 0; axis_index < kModeSwitchAxisCount; ++axis_index) {
        if (IsEyouPhModeSwitchAxis(axis_index)) {
            return true;
        }
    }

    return false;
}

bool CanEnterCstWithoutEnableChange(const CycleCache& cache, bool power_on_requested)
{
    return power_on_requested && AreAllModeSwitchAxesOperationEnabled(cache);
}

bool IsEyouCstToCspTransitionActive()
{
    return eyou_cst_to_csp_state != EyouCstToCspTransitionState::Idle;
}

bool IsEyouCstToCstTransitionActive()
{
    return eyou_cst_to_cst_state != EyouCstToCstTransitionState::Idle;
}

bool IsArmModeSwitchMonitorBypassed()
{
    return IsEyouCstToCstTransitionActive() ||
           IsEyouCstToCspTransitionActive() ||
           csp_reentry_hold_active ||
           csp_reentry_enable_pending;
}

std::size_t GetMaxDriveProfileDomainRegCount()
{
    std::size_t max_count = 0;
    for (const DriveProfile& profile : drive_profiles) {
        max_count = std::max(max_count, profile.domain_reg_count);
    }
    return max_count;
}

void SeedDesiredPositionFromFeedback(uint8_t axis_index, int32_t feedback_position)
{
    ec_shm_desire_axis_data.axis_ctr[axis_index].axis_position = feedback_position;
    ec_shm_desire_axis_data.axis_ctr[axis_index].axis_velocity = 0;

    if (ec_shm_desire_axis_data_ptr != nullptr) {
        ec_shm_desire_axis_data_ptr->axis_ctr[axis_index].axis_position = feedback_position;
        ec_shm_desire_axis_data_ptr->axis_ctr[axis_index].axis_velocity = 0;
    }
}

uint8_t CountConfiguredModeSwitchAxes()
{
    uint8_t count = 0;
    for (uint8_t axis_index = 0; axis_index < kModeSwitchAxisCount; ++axis_index) {
        if (mode_switch_axis_configured[axis_index]) {
            ++count;
        }
    }
    return count;
}

bool ResolveAxisIndexFromAlias(
    uint16_t alias,
    uint8_t& axis_index)
{
    if (TryResolveAxisIndex(alias, axis_index)) {
        return true;
    }

    axis_index = kInvalidAxisIndex;
    return false;
}

uint8_t NormalizeArmGroupMode(uint8_t requested_mode)
{
    return requested_mode == AXIS_MODE_CST ? AXIS_MODE_CST : AXIS_MODE_CSP;
}

const char* GetAxisModeName(uint8_t mode)
{
    switch (mode) {
    case AXIS_MODE_CSP:
        return "CSP";
    case AXIS_MODE_CST:
        return "CST";
    case AXIS_MODE_CSV:
        return "CSV";
    default:
        return "UNKNOWN";
    }
}

void SnapshotModeSwitchAxisPositions(const CycleCache& cache)
{
    for (uint8_t axis_index = 0; axis_index < kModeSwitchAxisCount; ++axis_index) {
        if (!mode_switch_axis_configured[axis_index]) {
            continue;
        }

        hold_positions[axis_index] = cache.act_pos[axis_index];
        hold_position_initialized[axis_index] = true;
    }
}

void SeedModeSwitchAxesDesiredPositionsFromHold()
{
    for (uint8_t axis_index = 0; axis_index < kModeSwitchAxisCount; ++axis_index) {
        if (!mode_switch_axis_configured[axis_index] || !hold_position_initialized[axis_index]) {
            continue;
        }

        SeedDesiredPositionFromFeedback(axis_index, hold_positions[axis_index]);
    }
}

void ClearArmGroupHoldContext()
{
    for (uint8_t axis_index = 0; axis_index < kModeSwitchAxisCount; ++axis_index) {
        if (!mode_switch_axis_configured[axis_index]) {
            continue;
        }

        hold_position_initialized[axis_index] = false;
        cst_takeover_active[axis_index] = false;
        cst_takeover_start_torque[axis_index] = 0;
        cst_takeover_cycles[axis_index] = 0;
    }
    arm_group_hold_active = false;
    eyou_cst_enable_state = EyouCstEnableState::Idle;
    eyou_cst_to_cst_state = EyouCstToCstTransitionState::Idle;
    eyou_cst_to_cst_cycles = 0;
    eyou_cst_to_csp_state = EyouCstToCspTransitionState::Idle;
    eyou_cst_to_csp_cycles = 0;
    csp_reentry_hold_active = false;
    csp_reentry_enable_pending = false;
    csp_reentry_enable_state = CspReentryEnableState::Idle;
    csp_reentry_ready_cycles = 0;
    csp_reentry_takeover_cycles = 0;
}

int16_t ComputeCstTakeoverEffort(uint8_t axis_index, int32_t requested_effort)
{
    if (!cst_takeover_active[axis_index]) {
        return static_cast<int16_t>(requested_effort);
    }

    const uint32_t current_cycle = cst_takeover_cycles[axis_index];
    cst_takeover_cycles[axis_index] = current_cycle + 1;
    if (current_cycle == 0) {
        return cst_takeover_start_torque[axis_index];
    }
    if (current_cycle >= kCstTakeoverBlendCycles) {
        cst_takeover_active[axis_index] = false;
        return static_cast<int16_t>(requested_effort);
    }

    const int32_t start_torque = cst_takeover_start_torque[axis_index];
    const int32_t diff = requested_effort - start_torque;
    const int32_t blended =
        start_torque + static_cast<int32_t>((static_cast<int64_t>(diff) * current_cycle) / kCstTakeoverBlendCycles);
    return static_cast<int16_t>(blended);
}

bool IsAxisStandstill(const CycleCache& cache, uint8_t axis_index)
{
    return IsValidAxisIndex(axis_index) &&
           AbsI32(cache.act_vel[axis_index]) < kStandstillVelocityThreshold;
}

bool AreAllModeSwitchAxesOperationEnabled(const CycleCache& cache)
{
    for (uint8_t axis_index = 0; axis_index < kModeSwitchAxisCount; ++axis_index) {
        if (!mode_switch_axis_configured[axis_index]) {
            continue;
        }

        if (cache.state[axis_index] != STATE_OPERATION_ENABLED) {
            return false;
        }
    }

    return true;
}

bool HaveAllModeSwitchAxesLeftOperationEnabled(const CycleCache& cache)
{
    for (uint8_t axis_index = 0; axis_index < kModeSwitchAxisCount; ++axis_index) {
        if (!mode_switch_axis_configured[axis_index]) {
            continue;
        }

        if (cache.state[axis_index] == STATE_OPERATION_ENABLED) {
            return false;
        }
    }

    return true;
}

bool AreAllModeSwitchAxesInMode(const CycleCache& cache, uint8_t mode)
{
    for (uint8_t axis_index = 0; axis_index < kModeSwitchAxisCount; ++axis_index) {
        if (!mode_switch_axis_configured[axis_index]) {
            continue;
        }

        if (cache.mode_word[axis_index] != mode) {
            return false;
        }
    }

    return true;
}

bool AreAllModeSwitchAxesStandstill(const CycleCache& cache)
{
    for (uint8_t axis_index = 0; axis_index < kModeSwitchAxisCount; ++axis_index) {
        if (!mode_switch_axis_configured[axis_index]) {
            continue;
        }

        if (!IsAxisStandstill(cache, axis_index)) {
            return false;
        }
    }

    return true;
}

bool AreAllModeSwitchAxesReadyToSwitchOn(const CycleCache& cache)
{
    for (uint8_t axis_index = 0; axis_index < kModeSwitchAxisCount; ++axis_index) {
        if (!IsEyouPhModeSwitchAxis(axis_index)) {
            continue;
        }

        if (cache.state[axis_index] != STATE_READY_TO_SWITCH_ON &&
            cache.state[axis_index] != STATE_SWITCHED_ON &&
            cache.state[axis_index] != STATE_OPERATION_ENABLED) {
            return false;
        }
    }

    return true;
}

bool AreAllModeSwitchAxesSwitchedOn(const CycleCache& cache)
{
    for (uint8_t axis_index = 0; axis_index < kModeSwitchAxisCount; ++axis_index) {
        if (!IsEyouPhModeSwitchAxis(axis_index)) {
            continue;
        }

        if (cache.state[axis_index] != STATE_SWITCHED_ON &&
            cache.state[axis_index] != STATE_OPERATION_ENABLED) {
            return false;
        }
    }

    return true;
}

bool AreAllModeSwitchAxesDesiredPositionsWithinWindow()
{
    for (uint8_t axis_index = 0; axis_index < kModeSwitchAxisCount; ++axis_index) {
        if (!mode_switch_axis_configured[axis_index]) {
            continue;
        }

        if (!hold_position_initialized[axis_index]) {
            return false;
        }

        const int32_t diff =
            ec_shm_desire_axis_data.axis_ctr[axis_index].axis_position - hold_positions[axis_index];
        if (AbsI32(diff) > CSP_REENTRY_COMMAND_WINDOW) {
            return false;
        }
    }

    return true;
}

void UpdateCspReentryHoldState(const CycleCache& cache)
{
    if (!csp_reentry_hold_active) {
        return;
    }

    if (AreAllModeSwitchAxesInMode(cache, AXIS_MODE_CSP) &&
        AreAllModeSwitchAxesOperationEnabled(cache) &&
        AreAllModeSwitchAxesStandstill(cache)) {
        ++csp_reentry_ready_cycles;
    } else {
        csp_reentry_ready_cycles = 0;
        csp_reentry_takeover_cycles = 0;
        return;
    }

    if (csp_reentry_ready_cycles < kStandstillRequiredCycles) {
        csp_reentry_takeover_cycles = 0;
        return;
    }

    if (AreAllModeSwitchAxesDesiredPositionsWithinWindow()) {
        ++csp_reentry_takeover_cycles;
    } else {
        csp_reentry_takeover_cycles = 0;
    }

    if (csp_reentry_takeover_cycles >= kCspReentryTakeoverStableCycles) {
        csp_reentry_hold_active = false;
        csp_reentry_enable_pending = false;
        csp_reentry_ready_cycles = 0;
        csp_reentry_takeover_cycles = 0;
        arm_group_hold_active = false;
        csp_reentry_enable_state = CspReentryEnableState::Idle;
        printf("arm_group_mode_switch: CSP reentry takeover accepted, release position hold\n");
    }
}

bool AreAllModeSwitchAxesModeSuccess()
{
    for (uint8_t axis_index = 0; axis_index < kModeSwitchAxisCount; ++axis_index) {
        if (!mode_switch_axis_configured[axis_index]) {
            continue;
        }

        if (axis_mode_states[axis_index] != AXIS_MODE_SUCCESS) {
            return false;
        }
    }

    return true;
}

bool HasAnyModeSwitchAxisFault(const CycleCache& cache)
{
    for (uint8_t axis_index = 0; axis_index < kModeSwitchAxisCount; ++axis_index) {
        if (!mode_switch_axis_configured[axis_index]) {
            continue;
        }

        if ((cache.state[axis_index] == STATE_FAULT) || ((cache.status_word[axis_index] & AXIS_STATUES_ERR) != 0)) {
            return true;
        }
    }

    return false;
}

const char* GetModeSwitchStateName(ModeSwitchTestState state)
{
    switch (state) {
    case ModeSwitchTestState::BootHoldCsp:
        return "BootHoldCsp";
    case ModeSwitchTestState::EnableAxis:
        return "EnableAxis";
    case ModeSwitchTestState::WaitStandstillInCsp:
        return "WaitStandstillInCsp";
    case ModeSwitchTestState::CspDwell:
        return "CspDwell";
    case ModeSwitchTestState::RequestCst:
        return "RequestCst";
    case ModeSwitchTestState::WaitCstAck:
        return "WaitCstAck";
    case ModeSwitchTestState::CstDwell:
        return "CstDwell";
    case ModeSwitchTestState::RequestCsp:
        return "RequestCsp";
    case ModeSwitchTestState::WaitCspAck:
        return "WaitCspAck";
    case ModeSwitchTestState::FaultHold:
        return "FaultHold";
    }

    return "Unknown";
}

void LogModeSwitchEvent(const char* event, const CycleCache& cache)
{
    uint32_t enabled_count = 0;
    uint32_t requested_mode_count = 0;
    int32_t max_abs_velocity = 0;
    const uint8_t configured_count = CountConfiguredModeSwitchAxes();

    for (uint8_t axis_index = 0; axis_index < kModeSwitchAxisCount; ++axis_index) {
        if (!mode_switch_axis_configured[axis_index]) {
            continue;
        }

        if (cache.state[axis_index] == STATE_OPERATION_ENABLED) {
            ++enabled_count;
        }
        if (cache.mode_word[axis_index] == mode_switch_test_ctx.requested_mode) {
            ++requested_mode_count;
        }
        max_abs_velocity = std::max(max_abs_velocity, AbsI32(cache.act_vel[axis_index]));
    }

    printf(
        "axes:[0-13] event:%s state:%s req_mode:0x%02x enabled:%u/%u in_req:%u/%u max_abs_vel:%d "
        "samples:a0{mode:0x%02x pos:%d vel:%d} a7{mode:0x%02x pos:%d vel:%d} a13{mode:0x%02x pos:%d vel:%d}\n",
        event,
        GetModeSwitchStateName(mode_switch_test_ctx.state),
        mode_switch_test_ctx.requested_mode,
        enabled_count,
        configured_count,
        requested_mode_count,
        configured_count,
        max_abs_velocity,
        cache.mode_word[kModeSwitchLogAxisFirst],
        cache.act_pos[kModeSwitchLogAxisFirst],
        cache.act_vel[kModeSwitchLogAxisFirst],
        cache.mode_word[kModeSwitchLogAxisMiddle],
        cache.act_pos[kModeSwitchLogAxisMiddle],
        cache.act_vel[kModeSwitchLogAxisMiddle],
        cache.mode_word[kModeSwitchLogAxisLast],
        cache.act_pos[kModeSwitchLogAxisLast],
        cache.act_vel[kModeSwitchLogAxisLast]);
}

void LogCspReentryEnableEvent(const char* event, const CycleCache& cache)
{
    printf(
        "arm_group_mode_switch: %s samples:"
        " a0{state:0x%04x mode:0x%02x cw:0x%04x}"
        " a7{state:0x%04x mode:0x%02x cw:0x%04x}"
        " a13{state:0x%04x mode:0x%02x cw:0x%04x}\n",
        event,
        cache.state[kModeSwitchLogAxisFirst],
        cache.mode_word[kModeSwitchLogAxisFirst],
        static_cast<uint16_t>(ec_shm_to_axis_data.axis_ctr[kModeSwitchLogAxisFirst].ec_ctrword),
        cache.state[kModeSwitchLogAxisMiddle],
        cache.mode_word[kModeSwitchLogAxisMiddle],
        static_cast<uint16_t>(ec_shm_to_axis_data.axis_ctr[kModeSwitchLogAxisMiddle].ec_ctrword),
        cache.state[kModeSwitchLogAxisLast],
        cache.mode_word[kModeSwitchLogAxisLast],
        static_cast<uint16_t>(ec_shm_to_axis_data.axis_ctr[kModeSwitchLogAxisLast].ec_ctrword));
}

void LogStandstillWaitOnce(const char* context, const CycleCache& cache)
{
    if (mode_switch_test_ctx.wait_standstill_announced) {
        return;
    }

    mode_switch_test_ctx.wait_standstill_announced = true;
    printf(
        "axes:[0-13] event:等待静止 context:%s sample_vel:[%d,%d,%d] threshold:%d\n",
        context,
        cache.act_vel[kModeSwitchLogAxisFirst],
        cache.act_vel[kModeSwitchLogAxisMiddle],
        cache.act_vel[kModeSwitchLogAxisLast],
        kStandstillVelocityThreshold);
}

void ResetStandstillTracking()
{
    mode_switch_test_ctx.standstill_cycles = 0;
    mode_switch_test_ctx.wait_standstill_announced = false;
}

bool HasAnyAxisErrorCode(const CycleCache& cache)
{
    for (uint8_t axis_index = 0; axis_index < kMaxZeroAxisCount; ++axis_index) {
        if (cache.axis_error_code[axis_index] != 0) {
            return true;
        }
    }

    return false;
}

void PrintAxisErrorCodeLog(const CycleCache& cache)
{
    bool found_error = false;
    printf("axis_error_codes:");

    for (uint8_t axis_index = 0; axis_index < kMaxZeroAxisCount; ++axis_index) {
        if (cache.axis_error_code[axis_index] == 0) {
            continue;
        }

        printf(" a%u{err:0x%04x}", axis_index, cache.axis_error_code[axis_index]);
        found_error = true;
    }

    if (!found_error) {
        printf(" none");
    }

    printf("\n");
}

bool CreateAxisErrorCodeRequest(MasterContext& ctx, uint8_t axis_index, ec_slave_config_t* slave_config)
{
    ec_sdo_request_t* request =
        ecrt_slave_config_create_sdo_request(slave_config, 0x603F, 0x00, sizeof(uint16_t));
    if (request == nullptr) {
        printf("%s axis:%u failed to create 0x603F SDO request\n",
               GetMasterName(ctx),
               axis_index);
        return false;
    }

    if (ecrt_sdo_request_timeout(request, kAxisErrorRequestTimeoutMs)) {
        printf("%s axis:%u failed to set 0x603F SDO timeout\n",
               GetMasterName(ctx),
               axis_index);
        return false;
    }

    ctx.axis_error_code_requests[axis_index] = request;
    return true;
}

void PublishAxisErrorCode(uint8_t axis_index, uint16_t axis_error_code)
{
    if (ec_shm_real_axis_data_ptr == nullptr || axis_index >= kAxisStateSlotCount) {
        return;
    }

    ec_shm_real_axis_data_ptr->axis_state[axis_index].axis_error_code = axis_error_code;
}

void PollAxisErrorCodeRequests(MasterContext& ctx, CycleCache& cache)
{
    if (ctx.configured_drive_count == 0) {
        return;
    }

    ++ctx.axis_error_poll_divider;
    if (ctx.axis_error_poll_divider < kAxisErrorPollPeriodCycles) {
        return;
    }
    ctx.axis_error_poll_divider = 0;

    for (std::size_t attempt = 0; attempt < kMaxZeroAxisCount; ++attempt) {
        const uint8_t axis_index =
            static_cast<uint8_t>((ctx.axis_error_poll_cursor + attempt) % kMaxZeroAxisCount);
        ec_sdo_request_t* request = ctx.axis_error_code_requests[axis_index];
        if (request == nullptr) {
            continue;
        }

        const ec_request_state_t request_state = ecrt_sdo_request_state(request);
        if (request_state == EC_REQUEST_BUSY) {
            continue;
        }

        if (request_state == EC_REQUEST_SUCCESS &&
            ecrt_sdo_request_data_size(request) >= sizeof(uint16_t)) {
            cache.axis_error_code[axis_index] = EC_READ_U16(ecrt_sdo_request_data(request));
            PublishAxisErrorCode(axis_index, cache.axis_error_code[axis_index]);
        }

        if (ecrt_sdo_request_read(request)) {
            printf("%s axis:%u failed to schedule 0x603F SDO read\n",
                   GetMasterName(ctx),
                   axis_index);
        }

        ctx.axis_error_poll_cursor = static_cast<uint8_t>((axis_index + 1) % kMaxZeroAxisCount);
        return;
    }
}

bool ProbeSingleMaster(MasterContext& ctx)
{
    ctx.handle = ecrt_request_master(ctx.master_index);
    if (ctx.handle == nullptr) {
        printf("Requesting %s failed\n", GetMasterName(ctx));
        return false;
    }

    ec_master_info_t master_info{};
    ecrt_master(ctx.handle, &master_info);
    ctx.slave_count = master_info.slave_count;
    LoadSlaveInfos(ctx);
    return true;
}

bool InspectMasterOperationalState(MasterContext& ctx, bool& operational_ok)
{
    if (ctx.slave_count == 0) {
        return true;
    }

    ecrt_master_receive(ctx.handle);

    const uint16_t count =
        std::min<uint16_t>(ctx.slave_count, static_cast<uint16_t>(ctx.slave_infos.size()));

    for (uint16_t i = 0; i < count; ++i) {
        if (ctx.slave_configs[i] == nullptr) {
            continue;
        }

        ecrt_slave_config_state(ctx.slave_configs[i], &ctx.slave_states[i]);
        if (ctx.slave_states[i].operational) {
            printf("%s slave:%d have operational_ok OP state\n", GetMasterName(ctx), i);
        } else {
            printf("%s slave:%d have operational_nook OP state\n", GetMasterName(ctx), i);
            operational_ok = false;
        }
    }

    return true;
}

} // namespace

/* 只负责读取从站描述信息，不做任何配置。 */
void LoadSlaveInfos(MasterContext& ctx)
{
    const uint16_t count =
        std::min<uint16_t>(ctx.slave_count, static_cast<uint16_t>(ctx.slave_infos.size()));

    for (uint16_t i = 0; i < count; ++i) {
        ecrt_master_get_slave(ctx.handle, i, &ctx.slave_infos[i]);
    }
}

/* 统一申请两个 master，并在这里完成 slave 总数检查。 */
bool RequestAndProbeMasters(MasterContext& master0, MasterContext& master1_ctx, uint16_t& totalSlaveCount)
{
    if (!ProbeSingleMaster(master0)) {
        return false;
    }

    master = master0.handle;

    if (!ProbeSingleMaster(master1_ctx)) {
        ReleaseMasterIfNeeded(master);
        master0.handle = nullptr;
        return false;
    }

    master1 = master1_ctx.handle;
    totalSlaveCount = static_cast<uint16_t>(master0.slave_count + master1_ctx.slave_count);

    if (master0.slave_count > kMaxMasterSlaveCount || master1_ctx.slave_count > kMaxMasterSlaveCount) {
        printf("Fail master slave_count exceeds local cache, master0:%u master1:%u max_per_master:%zu\n",
               master0.slave_count,
               master1_ctx.slave_count,
               kMaxMasterSlaveCount);
        ReleaseMasterIfNeeded(master);
        ReleaseMasterIfNeeded(master1);
        master0.handle = nullptr;
        master1_ctx.handle = nullptr;
        return false;
    }

    printf("master_info.slave_count: %d, axis range controlled by alias 0x%04x-0x%04x\n",
           totalSlaveCount,
           kAliasBase,
           static_cast<uint16_t>(kAliasBase + kMaxZeroAxisCount - 1));
    return true;
}

/* 绑定 slave config，并建立 local slave 到轴号的映射关系。 */
bool InitializeZeroDrives(MasterContext& ctx)
{
    ctx.drive_axis_map.fill(kInvalidAxisIndex);
    ctx.slave_uses_alias_selection.fill(false);
    ctx.zero_drive_configs.fill(nullptr);
    ctx.axis_error_code_requests.fill(nullptr);
    ctx.slave_configs.fill(nullptr);
    ctx.reference_drive = nullptr;
    ctx.configured_drive_count = 0;
    ctx.axis_error_poll_cursor = 0;
    ctx.axis_error_poll_divider = 0;

    const uint16_t count =
        std::min<uint16_t>(ctx.slave_count, static_cast<uint16_t>(ctx.slave_infos.size()));

    for (uint16_t i = 0; i < count; ++i) {
        const ec_slave_info_t& slave_info = ctx.slave_infos[i];
        const DriveProfile* drive_profile =
            FindDriveProfile(slave_info.vendor_id, slave_info.product_code);

        if (drive_profile == nullptr) {
            printf("%s_slaveid:%d Unsupported drive identity vendor=0x%08x product=0x%08x\n",
                   GetMasterName(ctx),
                   i,
                   slave_info.vendor_id,
                   slave_info.product_code);
            continue;
        }

        uint8_t axis_index = kInvalidAxisIndex;
        if (!ResolveAxisIndexFromAlias(slave_info.alias, axis_index)) {
            printf("%s slaveid:%d invalid alias:%u, expected alias range 0x%04x-0x%04x\n",
                   GetMasterName(ctx),
                   i,
                   slave_info.alias,
                   kAliasBase,
                   static_cast<uint16_t>(kAliasBase + kMaxZeroAxisCount - 1));
            continue;
        }

        if (IsAxisConfigured(axis_index)) {
            printf("%s slaveid:%d duplicate alias:0x%04x axis:%u already configured, skip\n",
                   GetMasterName(ctx),
                   i,
                   slave_info.alias,
                   axis_index);
            continue;
        }

        ec_slave_config_t* slave_config =
            ecrt_master_slave_config(
                ctx.handle,
                slave_info.alias,
                0,
                slave_info.vendor_id,
                slave_info.product_code);
        if (slave_config == nullptr) {
            printf("Failed to get slave_zero configuration: %d\n", axis_index);
            return false;
        }

        drive_profile->init(*drive_profile, ctx.handle, i, axis_index);

        ctx.slave_configs[i] = slave_config;
        ctx.slave_uses_alias_selection[i] = true;
        ctx.zero_drive_configs[axis_index] = slave_config;
        ctx.drive_axis_map[i] = axis_index;
        axis_drive_profiles[axis_index] = drive_profile->runtime_profile;
        if (IsModeSwitchAxis(axis_index) && drive_profile->supports_cst) {
            mode_switch_axis_configured[axis_index] = true;
        }

        if (!CreateAxisErrorCodeRequest(ctx, axis_index, slave_config)) {
            return false;
        }

        if (ctx.reference_drive == nullptr) {
            ctx.reference_drive = slave_config;
        }

        ++ctx.configured_drive_count;
        printf(
            "%s slave local:%d alias:0x%04x axis:%d profile:%s supports_cst:%u vendor=0x%08x product=0x%08x success to get mixed-drive configuration\n",
               GetMasterName(ctx),
               i,
               slave_info.alias,
               axis_index,
               drive_profile->name,
               static_cast<unsigned int>(drive_profile->supports_cst),
               slave_info.vendor_id,
               slave_info.product_code);
    }

    return true;
}

/* PDO 配置阶段独立出来，后续更换映射时不影响初始化逻辑。 */
bool ConfigureZeroPdos(MasterContext& ctx)
{
#ifdef CONFIG_PDOS
    const uint16_t count =
        std::min<uint16_t>(ctx.slave_count, static_cast<uint16_t>(ctx.slave_infos.size()));

    for (uint16_t i = 0; i < count; ++i) {
        const DriveProfile* drive_profile =
            FindDriveProfile(ctx.slave_infos[i].vendor_id, ctx.slave_infos[i].product_code);
        if (drive_profile == nullptr || ctx.slave_configs[i] == nullptr) {
            continue;
        }

        if (ecrt_slave_config_pdos(ctx.slave_configs[i], EC_END, drive_profile->syncs)) {
            printf("Failed to configure %s PDO mappings on %s slave:%d\n",
                   drive_profile->name,
                   GetMasterName(ctx),
                   i);
            return false;
        }

        printf("success to configure %s PDO mappings on %s slave:%d\n",
               drive_profile->name,
               GetMasterName(ctx),
               i);
    }
#endif

    return true;
}

/* 为每个 zero 轴生成 15 个 PDO offset 注册项。 */
std::vector<ec_pdo_entry_reg_t> BuildZeroDomainRegs(const MasterContext& ctx, AxisPdoOffsetTable& offsets)
{
    std::vector<ec_pdo_entry_reg_t> regs;
    regs.reserve(
        static_cast<std::size_t>(ctx.configured_drive_count) * GetMaxDriveProfileDomainRegCount() +
        1U);

    const uint16_t count =
        std::min<uint16_t>(ctx.slave_count, static_cast<uint16_t>(ctx.slave_infos.size()));

    for (uint16_t i = 0; i < count; ++i) {
        const DriveProfile* drive_profile =
            FindDriveProfile(ctx.slave_infos[i].vendor_id, ctx.slave_infos[i].product_code);
        if (drive_profile == nullptr) {
            continue;
        }

        const uint8_t axis_index = ctx.drive_axis_map[i];
        if (!IsValidAxisIndex(axis_index)) {
            continue;
        }

        const uint16_t reg_alias = ctx.slave_infos[i].alias;
        const uint16_t reg_position = 0;
        const uint32_t vendor_id = ctx.slave_infos[i].vendor_id;
        const uint32_t product_code = ctx.slave_infos[i].product_code;
        const DriveRuntimeProfile runtime_profile = drive_profile->runtime_profile;

        regs.push_back(ec_pdo_entry_reg_t{
            reg_alias, reg_position, vendor_id, product_code, 0x6040, 0x00,
            &offsets.controlword[axis_index]});
        regs.push_back(ec_pdo_entry_reg_t{
            reg_alias, reg_position, vendor_id, product_code, 0x607A, 0x00,
            &offsets.target_position[axis_index]});

        if (runtime_profile == DriveRuntimeProfile::ZeroLegacy ||
            runtime_profile == DriveRuntimeProfile::EyouPh) {
            regs.push_back(ec_pdo_entry_reg_t{
                reg_alias, reg_position, vendor_id, product_code, 0x60FF, 0x00,
                &offsets.target_velocity[axis_index]});
        }

        if (IsZeroLikeProfile(runtime_profile) || runtime_profile == DriveRuntimeProfile::EyouPh) {
            regs.push_back(ec_pdo_entry_reg_t{
                reg_alias, reg_position, vendor_id, product_code, 0x6071, 0x00,
                &offsets.target_torque[axis_index]});
        }

        if (runtime_profile == DriveRuntimeProfile::ZeroEyou) {
            regs.push_back(ec_pdo_entry_reg_t{
                reg_alias, reg_position, vendor_id, product_code, 0x60FE, 0x00,
                &offsets.digital_output[axis_index]});
        }

        regs.push_back(ec_pdo_entry_reg_t{
            reg_alias, reg_position, vendor_id, product_code, 0x6060, 0x00,
            &offsets.operation_mode[axis_index]});
        regs.push_back(ec_pdo_entry_reg_t{
            reg_alias, reg_position, vendor_id, product_code, 0x6041, 0x00,
            &offsets.statusword[axis_index]});
        regs.push_back(ec_pdo_entry_reg_t{
            reg_alias, reg_position, vendor_id, product_code, 0x6064, 0x00,
            &offsets.actual_position[axis_index]});
        regs.push_back(ec_pdo_entry_reg_t{
            reg_alias, reg_position, vendor_id, product_code, 0x606C, 0x00,
            &offsets.actual_velocity[axis_index]});
        regs.push_back(ec_pdo_entry_reg_t{
            reg_alias, reg_position, vendor_id, product_code, 0x6077, 0x00,
            &offsets.actual_torque[axis_index]});

        if (drive_profile->encoder_0_index != 0) {
            regs.push_back(ec_pdo_entry_reg_t{
                reg_alias, reg_position, vendor_id, product_code, drive_profile->encoder_0_index, 0x00,
                &offsets.actual_encoder_0[axis_index]});
        }
        if (drive_profile->encoder_1_index != 0) {
            regs.push_back(ec_pdo_entry_reg_t{
                reg_alias, reg_position, vendor_id, product_code, drive_profile->encoder_1_index, 0x00,
                &offsets.actual_encoder_1[axis_index]});
        }

        regs.push_back(ec_pdo_entry_reg_t{
            reg_alias, reg_position, vendor_id, product_code, 0x6061, 0x00,
            &offsets.actual_modestate[axis_index]});

        if (runtime_profile == DriveRuntimeProfile::EyouPh) {
            regs.push_back(ec_pdo_entry_reg_t{
                reg_alias, reg_position, vendor_id, product_code, 0x603F, 0x00,
                &offsets.axis_error_code[axis_index]});
        }
    }

    regs.push_back(ec_pdo_entry_reg_t{});
    return regs;
}

bool CreateAndRegisterDomain(MasterContext& ctx, const std::vector<ec_pdo_entry_reg_t>& regs)
{
    ctx.domain = ecrt_master_create_domain(ctx.handle);
    if (ctx.domain == nullptr) {
        printf("Failed to create domain for %s\n", GetMasterName(ctx));
        return false;
    }

    if (ecrt_domain_reg_pdo_entry_list(ctx.domain, regs.data())) {
        printf("%s PDO entry registration failed\n", GetMasterName(ctx));
        return false;
    }

    return true;
}

/* DC 配置单独收口，主流程就能保持阶段清晰。 */
void ConfigureDistributedClocksIfEnabled(MasterContext& ctx)
{
#ifdef CONFIG_DC
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    ecrt_master_application_time(ctx.handle, EC_NEWTIMEVAL2NANO(now));

    const uint16_t count =
        std::min<uint16_t>(ctx.slave_count, static_cast<uint16_t>(ctx.slave_infos.size()));

    for (uint16_t i = 0; i < count; ++i) {
        const DriveProfile* drive_profile =
            FindDriveProfile(ctx.slave_infos[i].vendor_id, ctx.slave_infos[i].product_code);
        if (drive_profile == nullptr || ctx.slave_configs[i] == nullptr) {
            continue;
        }

        ecrt_slave_config_dc(ctx.slave_configs[i], drive_profile->dc_assign_activate, PERIOD_NS, 0, 0, 0);
    }
#endif

#ifdef SYNC_REF_TO_MASTER
    struct timespec master_init_time;
    clock_gettime(CLOCK_MONOTONIC, &master_init_time);
    ecrt_master_application_time(ctx.handle, TIMESPEC2NS(master_init_time));
#endif

#ifdef SYNC_MASTER_TO_REF
    if (ctx.reference_drive != nullptr) {
        ctx.dc_sync.dc_start_time_ns = system_time_ns(ctx.dc_sync);
        ctx.dc_sync.dc_time_ns = ctx.dc_sync.dc_start_time_ns;
        ecrt_master_application_time(ctx.handle, ctx.dc_sync.dc_start_time_ns);

        if (ecrt_master_select_reference_clock(ctx.handle, ctx.reference_drive)) {
            printf("Selecting slave 0 as reference clock failed on %s!\n", GetMasterName(ctx));
        }
    }
#endif
}

bool ActivateMasterAndFetchDomainData(MasterContext& ctx)
{
    if (ctx.slave_count == 0) {
        return true;
    }

    if (ecrt_master_activate(ctx.handle)) {
        printf("Activating %s failed\n", GetMasterName(ctx));
        return false;
    }

    ctx.process_data = ecrt_domain_data(ctx.domain);
    if (ctx.process_data == nullptr) {
        printf("Fetching %s domain data failed\n", GetMasterName(ctx));
        return false;
    }

    return true;
}

/* 收包和 domain_process 总是成对出现，单独封装让循环更干净。 */
void ReceiveAndProcessDomains(MasterContext& ctx, CycleCache& cache)
{
    if (ctx.slave_count == 0) {
        return;
    }

    ecrt_master_receive(ctx.handle);

    if (ctx.domain != nullptr) {
        ecrt_domain_process(ctx.domain);
    }

#ifdef MEASURE_PERF
    ecrt_master_reference_clock_time(ctx.handle, &cache.t_cur);
#endif
}

/* 反馈统一回写到共享内存，APP 只读取整理后的状态。 */
void ReadAxisFeedbackFromMaster(const MasterContext& ctx, const AxisPdoOffsetTable& offsets, CycleCache& cache)
{
    if (ctx.process_data == nullptr || ec_shm_real_axis_data_ptr == nullptr) {
        return;
    }

    const uint16_t count =
        std::min<uint16_t>(ctx.slave_count, static_cast<uint16_t>(ctx.slave_infos.size()));

    for (uint16_t i = 0; i < count; ++i) {
        if (FindDriveProfile(ctx.slave_infos[i].vendor_id, ctx.slave_infos[i].product_code) == nullptr) {
            continue;
        }

        const uint8_t axis_index = ctx.drive_axis_map[i];
        if (!IsValidAxisIndex(axis_index)) {
            continue;
        }

        cache.act_pos[axis_index] =
            EC_READ_S32(ctx.process_data + offsets.actual_position[axis_index]);
        cache.pos_demand[axis_index] =
            IsOptionalOffsetRegistered(offsets.position_demand[axis_index])
                ? EC_READ_S32(ctx.process_data + offsets.position_demand[axis_index])
                : 0;
        cache.act_vel[axis_index] =
            EC_READ_S32(ctx.process_data + offsets.actual_velocity[axis_index]);
        cache.act_torque[axis_index] =
            EC_READ_S16(ctx.process_data + offsets.actual_torque[axis_index]);
        cache.encoder_0[axis_index] =
            IsOptionalOffsetRegistered(offsets.actual_encoder_0[axis_index])
                ? EC_READ_S32(ctx.process_data + offsets.actual_encoder_0[axis_index])
                : cache.act_pos[axis_index];
        cache.encoder_1[axis_index] =
            IsOptionalOffsetRegistered(offsets.actual_encoder_1[axis_index])
                ? EC_READ_S32(ctx.process_data + offsets.actual_encoder_1[axis_index])
                : cache.act_pos[axis_index];
        cache.mode_word[axis_index] =
            EC_READ_U8(ctx.process_data + offsets.actual_modestate[axis_index]);
        cache.status_word[axis_index] =
            EC_READ_U16(ctx.process_data + offsets.statusword[axis_index]);
        cache.state[axis_index] = getDriveState(cache.status_word[axis_index]);
        if (IsOptionalOffsetRegistered(offsets.axis_error_code[axis_index])) {
            cache.axis_error_code[axis_index] =
                EC_READ_U16(ctx.process_data + offsets.axis_error_code[axis_index]);
        }

        ec_shm_real_axis_data_ptr->axis_state[axis_index].axis_position = cache.act_pos[axis_index];
        ec_shm_real_axis_data_ptr->axis_state[axis_index].axis_velocity = cache.act_vel[axis_index];
        ec_shm_real_axis_data_ptr->axis_state[axis_index].axis_effort = cache.act_torque[axis_index];
        ec_shm_real_axis_data_ptr->axis_state[axis_index].axis_encoder_0 = cache.encoder_0[axis_index];
        ec_shm_real_axis_data_ptr->axis_state[axis_index].axis_encoder_1 = cache.encoder_1[axis_index];
        ec_shm_real_axis_data_ptr->axis_state[axis_index].axis_error_code =
            cache.axis_error_code[axis_index];
        ec_shm_real_axis_data_ptr->axis_state[axis_index].ec_ctrstate = cache.state[axis_index];
        ec_shm_real_axis_data_ptr->axis_state[axis_index].ec_modestate = cache.mode_word[axis_index];
    }
}

bool CheckAnyAxisFault(const CycleCache& cache)
{
    for (std::size_t i = 0; i < axis_drive_profiles.size(); ++i) {
        if (axis_drive_profiles[i] == DriveRuntimeProfile::None) {
            continue;
        }
        if (IsAxisFaulted(cache, static_cast<uint8_t>(i))) {
            return true;
        }
    }

    return false;
}

bool FindFirstSharedMotorNotOperationEnabled(
    uint8_t& abnormal_axis,
    uint8_t& abnormal_state,
    bool ignore_mode_switch_axes = false)
{
    abnormal_axis = kInvalidAxisIndex;
    abnormal_state = 0;

    if (ec_shm_real_axis_data_ptr == nullptr) {
        return false;
    }

    for (std::size_t i = 0; i < kSharedMotorStatusAxisCount && i < kAxisStateSlotCount; ++i) {
        if (ignore_mode_switch_axes && i < kModeSwitchAxisCount) {
            continue;
        }

        const uint8_t axis_state = ec_shm_real_axis_data_ptr->axis_state[i].ec_ctrstate;
        if (axis_state != STATE_OPERATION_ENABLED) {
            abnormal_axis = static_cast<uint8_t>(i);
            abnormal_state = axis_state;
            return true;
        }
    }

    return false;
}

bool AreAllSharedMotorsOperationEnabled()
{
    if (ec_shm_real_axis_data_ptr == nullptr) {
        return false;
    }

    for (std::size_t i = 0; i < kSharedMotorStatusAxisCount && i < kAxisStateSlotCount; ++i) {
        if (ec_shm_real_axis_data_ptr->axis_state[i].ec_ctrstate != STATE_OPERATION_ENABLED) {
            return false;
        }
    }

    return true;
}

const char* GetSharedMotorMonitorStateName(SharedMotorMonitorState state)
{
    switch (state) {
    case SharedMotorMonitorState::PowerOff:
        return "PowerOff";
    case SharedMotorMonitorState::Enabling:
        return "Enabling";
    case SharedMotorMonitorState::Armed:
        return "Armed";
    case SharedMotorMonitorState::Faulted:
        return "Faulted";
    }

    return "Unknown";
}

void PublishSharedMotorMonitorDebug(
    bool power_on_requested,
    bool all_shared_motors_enabled,
    bool has_abnormal_shared_axis_state,
    uint8_t abnormal_shared_axis,
    uint8_t abnormal_shared_state,
    bool any_axis_fault,
    bool mode_switch_bypass)
{
    if (shared_motor_monitor_debug_ptr == nullptr) {
        return;
    }

    shared_motor_monitor_debug_ptr->magic = kSharedMotorMonitorDebugMagic;
    shared_motor_monitor_debug_ptr->monitor_state =
        static_cast<uint8_t>(shared_motor_monitor_state);
    shared_motor_monitor_debug_ptr->force_shutdown =
        static_cast<uint8_t>(force_shutdown_all_zero_axes);
    shared_motor_monitor_debug_ptr->armed_latched =
        static_cast<uint8_t>(shared_motor_monitor_armed_latched);
    shared_motor_monitor_debug_ptr->seen_poweroff_after_fault =
        static_cast<uint8_t>(shared_motor_monitor_seen_poweroff_after_fault);
    shared_motor_monitor_debug_ptr->ec_poweron =
        static_cast<uint8_t>(power_on_requested);
    shared_motor_monitor_debug_ptr->power_request_allowed =
        static_cast<uint8_t>(power_on_requested && !force_shutdown_all_zero_axes);
    shared_motor_monitor_debug_ptr->all_20_enabled =
        static_cast<uint8_t>(all_shared_motors_enabled);
    shared_motor_monitor_debug_ptr->shared_bad =
        static_cast<uint8_t>(has_abnormal_shared_axis_state);
    shared_motor_monitor_debug_ptr->shared_bad_axis = abnormal_shared_axis;
    shared_motor_monitor_debug_ptr->shared_bad_state = abnormal_shared_state;
    shared_motor_monitor_debug_ptr->any_axis_fault =
        static_cast<uint8_t>(any_axis_fault);
    shared_motor_monitor_debug_ptr->mode_switch_bypass =
        static_cast<uint8_t>(mode_switch_bypass);
    ++shared_motor_monitor_debug_ptr->update_counter;
}

void PrintAxisDiagnosticLog(const MasterContext& ctx, const CycleCache& cache)
{
    const uint16_t count =
        std::min<uint16_t>(ctx.slave_count, static_cast<uint16_t>(ctx.slave_infos.size()));

    for (uint16_t i = 0; i < count; ++i) {
        if (FindDriveProfile(ctx.slave_infos[i].vendor_id, ctx.slave_infos[i].product_code) == nullptr) {
            continue;
        }

        const uint8_t axis_index = ctx.drive_axis_map[i];
        if (!IsValidAxisIndex(axis_index)) {
            continue;
        }

        printf(
            "%s diag axis:%u alias:%u 6041:0x%04x 6040:0x%04x 6061:0x%02x 603F:0x%04x\n",
            GetMasterName(ctx),
            axis_index,
            ctx.slave_infos[i].alias,
            cache.status_word[axis_index],
            static_cast<uint16_t>(ec_shm_to_axis_data.axis_ctr[axis_index].ec_ctrword),
            static_cast<uint8_t>(cache.mode_word[axis_index]),
            cache.axis_error_code[axis_index]);
    }
}

void PrintAxisPositionDiagnosticLog(const MasterContext& ctx, const CycleCache& cache)
{
    const uint16_t count =
        std::min<uint16_t>(ctx.slave_count, static_cast<uint16_t>(ctx.slave_infos.size()));

    for (uint16_t i = 0; i < count; ++i) {
        if (FindDriveProfile(ctx.slave_infos[i].vendor_id, ctx.slave_infos[i].product_code) == nullptr) {
            continue;
        }

        const uint8_t axis_index = ctx.drive_axis_map[i];
        if (!IsValidAxisIndex(axis_index)) {
            continue;
        }

        const int32_t actual_position = cache.act_pos[axis_index];
        const int32_t demand_position = cache.pos_demand[axis_index];
        const int32_t command_target_position = ec_shm_to_axis_data.axis_ctr[axis_index].axis_position;
        const int32_t app_target_position = ec_shm_desire_axis_data.axis_ctr[axis_index].axis_position;
        const int32_t position_diff = command_target_position - actual_position;
        const int32_t demand_diff = demand_position - actual_position;
        const bool use_hold_position =
            IsModeSwitchAxis(axis_index) && csp_reentry_hold_active && mode_switch_test_ctx.requested_mode == AXIS_MODE_CSP;

        printf(
            "%s posdiag axis:%u alias:%u actual:%d(0x%08x) demand:%d(0x%08x) demand_diff:%d target:%d(0x%08x) diff:%d app_target:%d(0x%08x) hold:%u state:0x%04x mode:0x%02x\n",
            GetMasterName(ctx),
            axis_index,
            ctx.slave_infos[i].alias,
            actual_position,
            static_cast<uint32_t>(actual_position),
            demand_position,
            static_cast<uint32_t>(demand_position),
            demand_diff,
            command_target_position,
            static_cast<uint32_t>(command_target_position),
            position_diff,
            app_target_position,
            static_cast<uint32_t>(app_target_position),
            static_cast<unsigned int>(use_hold_position),
            cache.state[axis_index],
            cache.mode_word[axis_index]);
    }
}

void PrintRuntimeSnapshotForMaster(
    const MasterContext& ctx,
    const AxisPdoOffsetTable& offsets,
    const CycleCache& cache)
{
    if (ctx.process_data == nullptr) {
        return;
    }

    const uint16_t count =
        std::min<uint16_t>(ctx.slave_count, static_cast<uint16_t>(ctx.slave_infos.size()));

    for (uint16_t i = 0; i < count; ++i) {
        const uint8_t axis_index = ctx.drive_axis_map[i];
        if (!IsValidAxisIndex(axis_index)) {
            continue;
        }

        const DriveProfile* drive_profile = GetAxisDriveProfile(axis_index);
        if (drive_profile == nullptr) {
            continue;
        }

        const uint16_t command_controlword =
            IsOptionalOffsetRegistered(offsets.controlword[axis_index])
                ? EC_READ_U16(ctx.process_data + offsets.controlword[axis_index])
                : 0;
        const uint8_t command_mode =
            IsOptionalOffsetRegistered(offsets.operation_mode[axis_index])
                ? EC_READ_U8(ctx.process_data + offsets.operation_mode[axis_index])
                : 0;

        printf(
            "%s runtime axis:%u profile:%s desired{mode:0x%02x cw:0x%04x pos:%d vel:%d tq:%d} "
            "cmd{mode:0x%02x cw:0x%04x pos:%d vel:%d tq:%d} "
            "fb{mode:0x%02x status:0x%04x pos:%d vel:%d tq:%d err:0x%04x}\n",
            GetMasterName(ctx),
            axis_index,
            drive_profile->name,
            static_cast<unsigned int>(ec_shm_desire_axis_data.axis_ctr[axis_index].ec_mode),
            static_cast<unsigned int>(ec_shm_desire_axis_data.axis_ctr[axis_index].ec_ctrword),
            ec_shm_desire_axis_data.axis_ctr[axis_index].axis_position,
            ec_shm_desire_axis_data.axis_ctr[axis_index].axis_velocity,
            ec_shm_desire_axis_data.axis_ctr[axis_index].axis_effort,
            static_cast<unsigned int>(command_mode),
            static_cast<unsigned int>(command_controlword),
            ec_shm_to_axis_data.axis_ctr[axis_index].axis_position,
            ec_shm_to_axis_data.axis_ctr[axis_index].axis_velocity,
            ec_shm_to_axis_data.axis_ctr[axis_index].axis_effort,
            static_cast<unsigned int>(cache.mode_word[axis_index]),
            static_cast<unsigned int>(cache.status_word[axis_index]),
            cache.act_pos[axis_index],
            cache.act_vel[axis_index],
            cache.act_torque[axis_index],
            cache.axis_error_code[axis_index]);
    }
}

void PrintRuntimeSnapshot(
    const MasterContext& master0,
    const MasterContext& master1_ctx,
    const AxisPdoOffsetTable& offsets,
    const CycleCache& cache)
{
    PrintRuntimeSnapshotForMaster(master0, offsets, cache);
    PrintRuntimeSnapshotForMaster(master1_ctx, offsets, cache);
}

/* 从共享内存拉取 APP 最新命令。 */
void RefreshDesiredCommandsFromSharedMemory()
{
    if (ec_shm_desire_axis_data_ptr == nullptr) {
        return;
    }

    std::memcpy(&ec_shm_desire_axis_data, ec_shm_desire_axis_data_ptr, sizeof(ec_app_read_reg_t));
}

bool IsDrivePowerOnRequested()
{
    return ec_shm_desire_axis_data.ec_poweron != 0 && !force_shutdown_all_zero_axes;
}

void ResetControlContextsForForcedShutdown()
{
    ClearArmGroupHoldContext();
    arm_group_power_was_requested = false;
    arm_group_prepared_this_cycle = false;
}

void UpdateSharedMotorMonitorState(const CycleCache& cache)
{
    const SharedMotorMonitorState previous_state = shared_motor_monitor_state;
    const bool power_on_requested = ec_shm_desire_axis_data.ec_poweron != 0;
    const bool power_on_rising_edge = power_on_requested && !shared_motor_monitor_previous_poweron;
    const bool mode_switch_bypass = IsArmModeSwitchMonitorBypassed();
    uint8_t abnormal_shared_axis = kInvalidAxisIndex;
    uint8_t abnormal_shared_state = 0;
    const bool has_abnormal_shared_axis_state =
        FindFirstSharedMotorNotOperationEnabled(
            abnormal_shared_axis,
            abnormal_shared_state,
            mode_switch_bypass);
    const bool all_shared_motors_enabled = AreAllSharedMotorsOperationEnabled();

    if (!power_on_requested) {
        shared_motor_monitor_armed_latched = false;
    } else if (all_shared_motors_enabled) {
        shared_motor_monitor_armed_latched = true;
    }

    switch (shared_motor_monitor_state) {
    case SharedMotorMonitorState::PowerOff:
        force_shutdown_all_zero_axes = false;
        if (power_on_rising_edge || power_on_requested) {
            shared_motor_monitor_state =
                shared_motor_monitor_armed_latched ? SharedMotorMonitorState::Armed
                                                   : SharedMotorMonitorState::Enabling;
        }
        break;

    case SharedMotorMonitorState::Enabling:
        /* 上使能过程中各轴会依次经过非 0x27 状态，这里只等待 20 轴全部进 0x27。 */
        force_shutdown_all_zero_axes = false;
        if (!power_on_requested) {
            shared_motor_monitor_state = SharedMotorMonitorState::PowerOff;
        } else if (shared_motor_monitor_armed_latched) {
            shared_motor_monitor_state = SharedMotorMonitorState::Armed;
        }
        break;

    case SharedMotorMonitorState::Armed:
        if (!power_on_requested) {
            shared_motor_monitor_state = SharedMotorMonitorState::PowerOff;
            force_shutdown_all_zero_axes = false;
        } else if ((shared_motor_monitor_armed_latched && has_abnormal_shared_axis_state) ||
                   cache.any_axis_fault) {
            shared_motor_monitor_state = SharedMotorMonitorState::Faulted;
            force_shutdown_all_zero_axes = true;
            shared_motor_monitor_seen_poweroff_after_fault = false;
        } else {
            force_shutdown_all_zero_axes = false;
        }
        break;

    case SharedMotorMonitorState::Faulted:
        /* 故障后保持锁定；必须先看到上层 poweron 回到 0，下一次 0->1 才允许重新上使能。 */
        force_shutdown_all_zero_axes = true;
        if (!power_on_requested) {
            shared_motor_monitor_seen_poweroff_after_fault = true;
        } else if (shared_motor_monitor_seen_poweroff_after_fault && power_on_rising_edge) {
            shared_motor_monitor_state =
                shared_motor_monitor_armed_latched ? SharedMotorMonitorState::Armed
                                                   : SharedMotorMonitorState::Enabling;
            force_shutdown_all_zero_axes = false;
        }
        break;
    }

    if (shared_motor_monitor_state == SharedMotorMonitorState::Faulted) {
        ResetControlContextsForForcedShutdown();
    }

    PublishSharedMotorMonitorDebug(
        power_on_requested,
        all_shared_motors_enabled,
        has_abnormal_shared_axis_state,
        abnormal_shared_axis,
        abnormal_shared_state,
        cache.any_axis_fault,
        mode_switch_bypass);

    if (shared_motor_monitor_state != previous_state) {
        printf(
            "shared_motor_monitor transition:%s->%s poweron:%u armed_latched:%u seen_poweroff_after_fault:%u mode_switch_bypass:%u all_20_enabled:%u shared_bad:%u shared_bad_axis:%u shared_bad_state:0x%02x any_fault:%u\n",
            GetSharedMotorMonitorStateName(previous_state),
            GetSharedMotorMonitorStateName(shared_motor_monitor_state),
            static_cast<unsigned int>(power_on_requested),
            static_cast<unsigned int>(shared_motor_monitor_armed_latched),
            static_cast<unsigned int>(shared_motor_monitor_seen_poweroff_after_fault),
            static_cast<unsigned int>(mode_switch_bypass),
            static_cast<unsigned int>(all_shared_motors_enabled),
            static_cast<unsigned int>(has_abnormal_shared_axis_state),
            static_cast<unsigned int>(abnormal_shared_axis),
            static_cast<unsigned int>(abnormal_shared_state),
            static_cast<unsigned int>(cache.any_axis_fault));
    }

    shared_motor_monitor_previous_poweron = power_on_requested;
}

void PrintEnableGateDiagnosticLog(const CycleCache& cache)
{
    const bool mode_switch_bypass = IsArmModeSwitchMonitorBypassed();
    uint8_t abnormal_shared_axis = kInvalidAxisIndex;
    uint8_t abnormal_shared_state = 0;
    const bool has_abnormal_shared_axis_state =
        FindFirstSharedMotorNotOperationEnabled(
            abnormal_shared_axis,
            abnormal_shared_state,
            mode_switch_bypass);

    printf(
        "enable_gate monitor_state:%s armed_latched:%u seen_poweroff_after_fault:%u mode_switch_bypass:%u ec_poweron:%u power_req:%u any_fault:%u force_shutdown:%u shared_bad:%u shared_bad_axis:%u shared_bad_state:0x%02x axis0_req_mode:0x%02x axis0_actual_state:0x%04x\n",
        GetSharedMotorMonitorStateName(shared_motor_monitor_state),
        static_cast<unsigned int>(shared_motor_monitor_armed_latched),
        static_cast<unsigned int>(shared_motor_monitor_seen_poweroff_after_fault),
        static_cast<unsigned int>(mode_switch_bypass),
        static_cast<unsigned int>(ec_shm_desire_axis_data.ec_poweron),
        static_cast<unsigned int>(IsDrivePowerOnRequested()),
        static_cast<unsigned int>(cache.any_axis_fault),
        static_cast<unsigned int>(force_shutdown_all_zero_axes),
        static_cast<unsigned int>(has_abnormal_shared_axis_state),
        static_cast<unsigned int>(abnormal_shared_axis),
        static_cast<unsigned int>(abnormal_shared_state),
        static_cast<unsigned int>(ec_shm_desire_axis_data.axis_ctr[0].ec_mode),
        static_cast<unsigned int>(cache.state[0]));
}

void ApplyAxisEnableGate(uint8_t axis_index, uint16_t drive_state)
{
    if (IsDrivePowerOnRequested()) {
        axis_ctrword_state_chage(drive_state, axis_index);
    } else {
        ec_shm_to_axis_data.axis_ctr[axis_index].ec_ctrword = CONTROL_WORD_SHUTDOWN;
    }
}

void PrepareForcedShutdownAxisCommand(uint8_t axis_index, const CycleCache& cache)
{
    uint8_t axis_command_mode = cache.mode_word[axis_index];
    if (axis_command_mode != AXIS_MODE_CSP &&
        axis_command_mode != AXIS_MODE_CST &&
        axis_command_mode != AXIS_MODE_CSV) {
        axis_command_mode = AXIS_MODE_CSP;
    }

    ec_shm_to_axis_data.axis_ctr[axis_index].ec_mode = axis_command_mode;
    ec_shm_to_axis_data.axis_ctr[axis_index].axis_position = cache.act_pos[axis_index];
    ec_shm_to_axis_data.axis_ctr[axis_index].axis_velocity = 0;
    ec_shm_to_axis_data.axis_ctr[axis_index].axis_effort = 0;
    ec_shm_to_axis_data.axis_ctr[axis_index].ec_ctrword = CONTROL_WORD_SHUTDOWN;
    axis_mode_states[axis_index] = AXIS_MODE_SUCCESS;
}

void PrepareFixedCspAxisCommand(uint8_t axis_index, const CycleCache& cache)
{
    ec_shm_to_axis_data.axis_ctr[axis_index].ec_mode = AXIS_MODE_CSP;
    ec_shm_to_axis_data.axis_ctr[axis_index].axis_position =
        ec_shm_desire_axis_data.axis_ctr[axis_index].axis_position;
    ec_shm_to_axis_data.axis_ctr[axis_index].axis_velocity = 0;
    ec_shm_to_axis_data.axis_ctr[axis_index].axis_effort = 0;

    axis_mode_state_chage(
        cache.mode_word[axis_index],
        AXIS_MODE_CSP,
        axis_index,
        cache.state[axis_index],
        cache.act_pos[axis_index],
        ec_shm_desire_axis_data.axis_ctr[axis_index].axis_position,
        cache.act_vel[axis_index],
        0);

    ApplyAxisEnableGate(axis_index, cache.state[axis_index]);
}

void ApplyModeSwitchGroupOutputs(const CycleCache& cache)
{
    for (uint8_t axis_index = 0; axis_index < kModeSwitchAxisCount; ++axis_index) {
        if (!mode_switch_axis_configured[axis_index]) {
            continue;
        }

        ec_shm_to_axis_data.axis_ctr[axis_index].ec_mode = mode_switch_test_ctx.requested_mode;
        ec_shm_to_axis_data.axis_ctr[axis_index].axis_position = hold_positions[axis_index];
        ec_shm_to_axis_data.axis_ctr[axis_index].axis_velocity = 0;
        ec_shm_to_axis_data.axis_ctr[axis_index].axis_effort = 0;

        axis_mode_state_chage(
            cache.mode_word[axis_index],
            mode_switch_test_ctx.requested_mode,
            axis_index,
            cache.state[axis_index],
            cache.act_pos[axis_index],
            hold_positions[axis_index],
            cache.act_vel[axis_index],
            0);

        axis_ctrword_state_chage(cache.state[axis_index], axis_index);
    }
}

void PrepareArmGroupAxisCommand(const CycleCache& cache)
{
    uint8_t requested_mode = NormalizeArmGroupMode(ec_shm_desire_axis_data.axis_ctr[0].ec_mode);
    const bool power_on_requested = IsDrivePowerOnRequested();
    const bool power_on_rising_edge = power_on_requested && !arm_group_power_was_requested;
    const bool has_eyou_mode_switch_axes = HasAnyEyouPhModeSwitchAxis();
    const bool cst_request_blocked = requested_mode == AXIS_MODE_CST && !power_on_requested;

    if (cst_request_blocked) {
        requested_mode = AXIS_MODE_CSP;
    }

    arm_group_requested_mode = requested_mode;

    if (requested_mode != arm_group_previous_mode) {
        printf(
            "arm_group_mode_switch: %s -> %s axis0_cmd:0x%02x actual_modes[a0,a7,a13]=[0x%02x,0x%02x,0x%02x]\n",
            GetAxisModeName(arm_group_previous_mode),
            GetAxisModeName(requested_mode),
            ec_shm_desire_axis_data.axis_ctr[0].ec_mode,
            cache.mode_word[kModeSwitchLogAxisFirst],
            cache.mode_word[kModeSwitchLogAxisMiddle],
            cache.mode_word[kModeSwitchLogAxisLast]);
    }

    if (!power_on_requested) {
        ClearArmGroupHoldContext();
        arm_group_power_was_requested = false;
    }

    if (power_on_rising_edge) {
        arm_group_hold_active = false;
        eyou_cst_enable_state = EyouCstEnableState::Idle;
        eyou_cst_to_cst_state = EyouCstToCstTransitionState::Idle;
        eyou_cst_to_cst_cycles = 0;
        eyou_cst_to_csp_state = EyouCstToCspTransitionState::Idle;
        eyou_cst_to_csp_cycles = 0;
        csp_reentry_hold_active = false;
        csp_reentry_enable_pending = false;
        csp_reentry_enable_state = CspReentryEnableState::Idle;
        csp_reentry_ready_cycles = 0;
        csp_reentry_takeover_cycles = 0;
    }

    const bool entering_cst =
        requested_mode == AXIS_MODE_CST &&
        (arm_group_previous_mode != AXIS_MODE_CST || !arm_group_hold_active);
    const bool leaving_cst =
        requested_mode == AXIS_MODE_CSP &&
        arm_group_previous_mode == AXIS_MODE_CST &&
        power_on_requested;

    if (requested_mode == AXIS_MODE_CST) {
        if (entering_cst) {
            SnapshotModeSwitchAxisPositions(cache);
            SeedModeSwitchAxesDesiredPositionsFromHold();
            arm_group_hold_active = true;
            eyou_cst_enable_state = EyouCstEnableState::Idle;
            eyou_cst_to_cst_state =
                has_eyou_mode_switch_axes ? EyouCstToCstTransitionState::DisableHold
                                          : EyouCstToCstTransitionState::Idle;
            eyou_cst_to_cst_cycles = 0;
            eyou_cst_to_csp_state = EyouCstToCspTransitionState::Idle;
            eyou_cst_to_csp_cycles = 0;
            csp_reentry_hold_active = false;
            csp_reentry_enable_pending = false;
            csp_reentry_enable_state = CspReentryEnableState::Idle;
            csp_reentry_ready_cycles = 0;
            csp_reentry_takeover_cycles = 0;

            LogCspReentryEnableEvent(
                "CSP->CST allowed while power-on and already enabled, keep enable state unchanged",
                cache);
        }

        if (has_eyou_mode_switch_axes) {
            switch (eyou_cst_to_cst_state) {
            case EyouCstToCstTransitionState::DisableHold:
                ++eyou_cst_to_cst_cycles;
                if (eyou_cst_to_cst_cycles >= kEyouCstToCstDisableHoldCycles) {
                    eyou_cst_to_cst_state = EyouCstToCstTransitionState::ModeHold;
                    eyou_cst_to_cst_cycles = 0;
                    LogCspReentryEnableEvent(
                        "CSP->CST disable hold complete, switch PDO mode to CST",
                        cache);
                }
                break;
            case EyouCstToCstTransitionState::ModeHold:
                ++eyou_cst_to_cst_cycles;
                if (eyou_cst_to_cst_cycles >= kEyouCstToCstModeHoldCycles) {
                    eyou_cst_to_cst_state = EyouCstToCstTransitionState::ReEnable;
                    eyou_cst_to_cst_cycles = 0;
                    LogCspReentryEnableEvent(
                        "CSP->CST CST mode hold complete, start re-enable",
                        cache);
                }
                break;
            case EyouCstToCstTransitionState::ReEnable:
                if (AreAllModeSwitchAxesInMode(cache, AXIS_MODE_CST) &&
                    AreAllModeSwitchAxesOperationEnabled(cache)) {
                    eyou_cst_to_cst_state = EyouCstToCstTransitionState::Idle;
                    LogCspReentryEnableEvent("CSP->CST re-enable accepted", cache);
                }
                break;
            case EyouCstToCstTransitionState::Idle:
                break;
            }
        }
    } else {
        if (leaving_cst) {
            SnapshotModeSwitchAxisPositions(cache);
            SeedModeSwitchAxesDesiredPositionsFromHold();
            arm_group_hold_active = false;
            eyou_cst_enable_state = EyouCstEnableState::Idle;
            csp_reentry_hold_active = has_eyou_mode_switch_axes;
            csp_reentry_enable_pending = has_eyou_mode_switch_axes;
            csp_reentry_enable_state = CspReentryEnableState::Idle;
            csp_reentry_ready_cycles = 0;
            csp_reentry_takeover_cycles = 0;
            if (has_eyou_mode_switch_axes) {
                eyou_cst_to_csp_state = EyouCstToCspTransitionState::DisableHold;
                eyou_cst_to_csp_cycles = 0;
                LogCspReentryEnableEvent("CST->CSP drop enable begin", cache);
            } else {
                eyou_cst_to_csp_state = EyouCstToCspTransitionState::Idle;
                eyou_cst_to_csp_cycles = 0;
            }
        }

        if (has_eyou_mode_switch_axes) {
            switch (eyou_cst_to_csp_state) {
            case EyouCstToCspTransitionState::DisableHold:
                ++eyou_cst_to_csp_cycles;
                if (eyou_cst_to_csp_cycles >= kEyouCstToCspDisableHoldCycles) {
                    eyou_cst_to_csp_state = EyouCstToCspTransitionState::ModeHold;
                    eyou_cst_to_csp_cycles = 0;
                    LogCspReentryEnableEvent(
                        "CST->CSP disable hold complete, switch PDO mode to CSP",
                        cache);
                }
                break;
            case EyouCstToCspTransitionState::ModeHold:
                ++eyou_cst_to_csp_cycles;
                if (eyou_cst_to_csp_cycles >= kEyouCstToCspModeHoldCycles) {
                    eyou_cst_to_csp_state = EyouCstToCspTransitionState::ReEnable;
                    eyou_cst_to_csp_cycles = 0;
                    LogCspReentryEnableEvent(
                        "CST->CSP CSP mode hold complete, start re-enable",
                        cache);
                }
                break;
            case EyouCstToCspTransitionState::ReEnable:
                UpdateCspReentryHoldState(cache);
                if (!csp_reentry_hold_active &&
                    AreAllModeSwitchAxesInMode(cache, AXIS_MODE_CSP) &&
                    AreAllModeSwitchAxesOperationEnabled(cache)) {
                    eyou_cst_to_csp_state = EyouCstToCspTransitionState::Idle;
                    csp_reentry_enable_pending = false;
                    LogCspReentryEnableEvent("CSP reentry accepted", cache);
                }
                break;
            case EyouCstToCspTransitionState::Idle:
                UpdateCspReentryHoldState(cache);
                break;
            }
        }

        if (requested_mode != AXIS_MODE_CST) {
            eyou_cst_to_cst_state = EyouCstToCstTransitionState::Idle;
            eyou_cst_to_cst_cycles = 0;
        }
    }

    const bool eyou_cst_to_cst_disable_hold =
        requested_mode == AXIS_MODE_CST &&
        eyou_cst_to_cst_state == EyouCstToCstTransitionState::DisableHold;
    const bool eyou_cst_to_cst_mode_hold =
        requested_mode == AXIS_MODE_CST &&
        eyou_cst_to_cst_state == EyouCstToCstTransitionState::ModeHold;
    const bool eyou_cst_to_cst_reenable =
        requested_mode == AXIS_MODE_CST &&
        eyou_cst_to_cst_state == EyouCstToCstTransitionState::ReEnable;
    const bool eyou_shutdown_before_csp_mode =
        requested_mode == AXIS_MODE_CSP &&
        eyou_cst_to_csp_state == EyouCstToCspTransitionState::DisableHold;
    const bool eyou_shutdown_after_csp_mode =
        requested_mode == AXIS_MODE_CSP &&
        eyou_cst_to_csp_state == EyouCstToCspTransitionState::ModeHold;
    const bool force_csp_reentry_shutdown =
        eyou_shutdown_before_csp_mode || eyou_shutdown_after_csp_mode;

    for (uint8_t axis_index = 0; axis_index < kModeSwitchAxisCount; ++axis_index) {
        if (!mode_switch_axis_configured[axis_index]) {
            continue;
        }

        const bool is_eyou_axis = IsEyouPhModeSwitchAxis(axis_index);
        uint8_t axis_command_mode = requested_mode;
        if (is_eyou_axis && eyou_cst_to_cst_disable_hold) {
            axis_command_mode = AXIS_MODE_CSP;
        } else if (is_eyou_axis &&
                   (eyou_cst_to_cst_mode_hold || eyou_cst_to_cst_reenable)) {
            axis_command_mode = AXIS_MODE_CST;
        } else if (is_eyou_axis && eyou_shutdown_before_csp_mode) {
            axis_command_mode = AXIS_MODE_CST;
        }

        ec_shm_to_axis_data.axis_ctr[axis_index].ec_mode = axis_command_mode;

        if (is_eyou_axis && eyou_cst_to_cst_disable_hold) {
            ec_shm_to_axis_data.axis_ctr[axis_index].axis_position =
                hold_position_initialized[axis_index] ? hold_positions[axis_index] : cache.act_pos[axis_index];
            ec_shm_to_axis_data.axis_ctr[axis_index].axis_velocity = 0;
            ec_shm_to_axis_data.axis_ctr[axis_index].axis_effort = 0;

            axis_mode_state_chage(
                cache.mode_word[axis_index],
                axis_command_mode,
                axis_index,
                cache.state[axis_index],
                cache.act_pos[axis_index],
                ec_shm_to_axis_data.axis_ctr[axis_index].axis_position,
                cache.act_vel[axis_index],
                0);
        } else if (axis_command_mode == AXIS_MODE_CST) {
            const int32_t hold_position =
                hold_position_initialized[axis_index] ? hold_positions[axis_index] : cache.act_pos[axis_index];

            ec_shm_to_axis_data.axis_ctr[axis_index].axis_position = hold_position;
            ec_shm_to_axis_data.axis_ctr[axis_index].axis_velocity = 0;
            ec_shm_to_axis_data.axis_ctr[axis_index].axis_effort =
                is_eyou_axis && eyou_cst_to_cst_mode_hold ? 0
                                                          : ec_shm_desire_axis_data.axis_ctr[axis_index].axis_effort;

            axis_mode_state_chage(
                cache.mode_word[axis_index],
                axis_command_mode,
                axis_index,
                cache.state[axis_index],
                cache.act_pos[axis_index],
                hold_position,
                cache.act_vel[axis_index],
                0);
        } else {
            const bool use_hold_position = csp_reentry_hold_active;
            const int32_t csp_position = use_hold_position && hold_position_initialized[axis_index]
                                             ? hold_positions[axis_index]
                                             : ec_shm_desire_axis_data.axis_ctr[axis_index].axis_position;

            ec_shm_to_axis_data.axis_ctr[axis_index].axis_position = csp_position;
            ec_shm_to_axis_data.axis_ctr[axis_index].axis_velocity =
                use_hold_position ? 0 : ec_shm_desire_axis_data.axis_ctr[axis_index].axis_velocity;
            ec_shm_to_axis_data.axis_ctr[axis_index].axis_effort = 0;

            axis_mode_state_chage(
                cache.mode_word[axis_index],
                AXIS_MODE_CSP,
                axis_index,
                cache.state[axis_index],
                cache.act_pos[axis_index],
                csp_position,
                cache.act_vel[axis_index],
                use_hold_position ? 0 : ec_shm_desire_axis_data.axis_ctr[axis_index].axis_velocity);
        }

        if (is_eyou_axis && (eyou_cst_to_cst_disable_hold || eyou_cst_to_cst_mode_hold)) {
            ec_shm_to_axis_data.axis_ctr[axis_index].ec_ctrword = CONTROL_WORD_SHUTDOWN;
        } else if (requested_mode == AXIS_MODE_CST) {
            ApplyAxisEnableGate(axis_index, cache.state[axis_index]);
        } else if (force_csp_reentry_shutdown && is_eyou_axis) {
            ec_shm_to_axis_data.axis_ctr[axis_index].ec_ctrword = CONTROL_WORD_SHUTDOWN;
        } else {
            ApplyAxisEnableGate(axis_index, cache.state[axis_index]);
        }
    }

    arm_group_previous_mode = requested_mode;
    arm_group_power_was_requested = power_on_requested;
    mode_switch_test_ctx.requested_mode = requested_mode;
}

/* 前14轴由 axis0.ec_mode 外部触发整组 CSP/CST 切换。 */
void UpdateAxisCommandState(uint8_t axis_index, const CycleCache& cache, bool state_is_clean)
{
    (void)state_is_clean;

    if (axis_index >= axis_mode_states.size()) {
        return;
    }

    const DriveProfile* drive_profile = GetAxisDriveProfile(axis_index);
    if (drive_profile == nullptr) {
        return;
    }

    if (force_shutdown_all_zero_axes) {
        PrepareForcedShutdownAxisCommand(axis_index, cache);
        return;
    }

    if (IsCstModeSwitchAxis(axis_index)) {
        if (!arm_group_prepared_this_cycle) {
            PrepareArmGroupAxisCommand(cache);
            arm_group_prepared_this_cycle = true;
        }
        return;
    }

    PrepareFixedCspAxisCommand(axis_index, cache);
}

void WriteAxisCommandToDomain(uint8_t* process_data, const AxisPdoOffsetTable& offsets, uint8_t axis_index)
{
    if (process_data == nullptr || !IsValidAxisIndex(axis_index)) {
        return;
    }

    const DriveProfile* drive_profile = GetAxisDriveProfile(axis_index);
    if (drive_profile == nullptr) {
        return;
    }

    /* 6060/6040 始终通过 PDO 下发，命令源由上层共享内存和当前轴模式决定。 */
    EC_WRITE_U8(
        process_data + offsets.operation_mode[axis_index],
        ec_shm_to_axis_data.axis_ctr[axis_index].ec_mode);
    EC_WRITE_U16(
        process_data + offsets.controlword[axis_index],
        ec_shm_to_axis_data.axis_ctr[axis_index].ec_ctrword);
    const DriveRuntimeProfile runtime_profile = drive_profile->runtime_profile;
    const bool has_target_velocity =
        runtime_profile == DriveRuntimeProfile::ZeroLegacy ||
        runtime_profile == DriveRuntimeProfile::EyouPh;

    if (IsOptionalOffsetRegistered(offsets.digital_output[axis_index])) {
        EC_WRITE_U32(process_data + offsets.digital_output[axis_index], 0);
    }
    if (IsOptionalOffsetRegistered(offsets.torque_offset[axis_index])) {
        EC_WRITE_S16(process_data + offsets.torque_offset[axis_index], 0);
    }
    if (IsOptionalOffsetRegistered(offsets.target_torque[axis_index])) {
        EC_WRITE_S16(
            process_data + offsets.target_torque[axis_index],
            static_cast<int16_t>(ec_shm_to_axis_data.axis_ctr[axis_index].axis_effort));
    }

    if (axis_mode_states[axis_index] == AXIS_MODE_SUCCESS) {
        if (has_target_velocity) {
            EC_WRITE_S32(
                process_data + offsets.target_velocity[axis_index],
                ec_shm_to_axis_data.axis_ctr[axis_index].ec_mode == AXIS_MODE_CSV
                    ? ec_shm_to_axis_data.axis_ctr[axis_index].axis_velocity
                    : 0);
        }
        if (IsOptionalOffsetRegistered(offsets.velocity_offset[axis_index])) {
            EC_WRITE_S32(
                process_data + offsets.velocity_offset[axis_index],
                ec_shm_to_axis_data.axis_ctr[axis_index].axis_velocity);
        }
        EC_WRITE_U32(
            process_data + offsets.target_position[axis_index],
            ec_shm_to_axis_data.axis_ctr[axis_index].axis_position);
    } else {
        /* 模式尚未就绪时，不发送激进的运动指令，先保持安全占位值。 */
        if (has_target_velocity) {
            EC_WRITE_S32(process_data + offsets.target_velocity[axis_index], 0);
        }
        if (IsOptionalOffsetRegistered(offsets.velocity_offset[axis_index])) {
            EC_WRITE_S32(process_data + offsets.velocity_offset[axis_index], 0);
        }
        EC_WRITE_U32(
            process_data + offsets.target_position[axis_index],
            ec_shm_to_axis_data.axis_ctr[axis_index].axis_position);
    }
}

/* 双 master 的 PDO 下发统一走这里。 */
void WriteMasterOutputs(const MasterContext& ctx, const AxisPdoOffsetTable& offsets, CycleCache& cache)
{
    if (ctx.process_data == nullptr) {
        return;
    }

    const bool state_is_clean = !cache.any_axis_fault;
    const uint16_t count =
        std::min<uint16_t>(ctx.slave_count, static_cast<uint16_t>(ctx.slave_infos.size()));

    for (uint16_t i = 0; i < count; ++i) {
        if (FindDriveProfile(ctx.slave_infos[i].vendor_id, ctx.slave_infos[i].product_code) == nullptr) {
            continue;
        }

        const uint8_t axis_index = ctx.drive_axis_map[i];
        if (!IsValidAxisIndex(axis_index)) {
            continue;
        }

        UpdateAxisCommandState(axis_index, cache, state_is_clean);
        WriteAxisCommandToDomain(ctx.process_data, offsets, axis_index);
    }
}

/* 同步与发送共用一层封装，OP 等待和正式循环都能复用。 */
void SyncAndSendMaster(MasterContext& ctx)
{
    if (ctx.slave_count == 0) {
        return;
    }

    if (ctx.domain != nullptr) {
        ecrt_domain_queue(ctx.domain);
    }

#ifdef SYNC_REF_TO_MASTER
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    ecrt_master_application_time(ctx.handle, TIMESPEC2NS(now));
    ecrt_master_sync_reference_clock(ctx.handle);
    ecrt_master_sync_slave_clocks(ctx.handle);
#endif

#ifdef SYNC_MASTER_TO_REF
    sync_distributed_clocks(ctx);
#endif

    ecrt_master_send(ctx.handle);

#ifdef SYNC_MASTER_TO_REF
    update_master_clock(ctx);
#endif
}

bool WaitUntilAllSlavesOperational(MasterContext& master0, MasterContext& master1_ctx)
{
    struct timespec wakeupTime;
    struct timespec cycleTime = {0, PERIOD_NS};
    clock_gettime(CLOCK_MONOTONIC, &wakeupTime);

    while (1) {
        bool operational_ok = true;

        timespec_add(&wakeupTime, &wakeupTime, &cycleTime);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeupTime, nullptr);

        InspectMasterOperationalState(master0, operational_ok);
        InspectMasterOperationalState(master1_ctx, operational_ok);

        if (operational_ok) {
            if (ec_shm_real_axis_data_ptr != nullptr) {
                ec_shm_real_axis_data_ptr->ec_powerstate = 1;
            }
            printf("ALL slaves have reached OP state\n");
            return true;
        }

        SyncAndSendMaster(master0);
        SyncAndSendMaster(master1_ctx);
    }
}

/* 实时循环主流程：
 * 1. 定时唤醒
 * 2. 收包处理
 * 3. 刷新反馈
 * 4. 读取 APP 命令
 * 5. 运行状态机并写出 PDO
 * 6. 同步时钟并发包
 */
void RunCyclicLoop(MasterContext& master0, MasterContext& master1_ctx, const AxisPdoOffsetTable& offsets)
{
    CycleCache cache;
    uint32_t time_log_flag = 0;
    uint32_t axis_error_log_counter = 0;
    uint32_t axis_diagnostic_log_counter = 0;

    struct timespec wakeupTime;
    struct timespec cycleTime = {0, PERIOD_NS};
    struct timespec sleepTime = cycleTime;
    clock_gettime(CLOCK_MONOTONIC, &wakeupTime);

#ifdef MEASURE_TIMING
    struct timespec currentLoopTime{};
    struct timespec previousLoopTime = wakeupTime;
    struct timespec intervalTime{};
#endif

    while (1) {
        timespec_add(&wakeupTime, &wakeupTime, &sleepTime);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeupTime, nullptr);

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_MONOTONIC, &currentLoopTime);
        timespec_sub(&intervalTime, &currentLoopTime, &previousLoopTime);
        if ((intervalTime.tv_nsec > 1100000) ||
            (intervalTime.tv_nsec < 900000) ||
            (time_log_flag == 1)) {
            printf("testtimesub.tv_nsec time: %lu ns\n", intervalTime.tv_nsec);
        }
        previousLoopTime = currentLoopTime;
#endif

        ReceiveAndProcessDomains(master0, cache);
        ReceiveAndProcessDomains(master1_ctx, cache);

        ReadAxisFeedbackFromMaster(master0, offsets, cache);
        ReadAxisFeedbackFromMaster(master1_ctx, offsets, cache);
        PollAxisErrorCodeRequests(master0, cache);
        PollAxisErrorCodeRequests(master1_ctx, cache);

        RefreshDesiredCommandsFromSharedMemory();
        cache.any_axis_fault = CheckAnyAxisFault(cache);
        UpdateSharedMotorMonitorState(cache);
        arm_group_prepared_this_cycle = false;

        WriteMasterOutputs(master0, offsets, cache);
        WriteMasterOutputs(master1_ctx, offsets, cache);

        if (++axis_error_log_counter >= kAxisErrorLogPeriodCycles) {
            axis_error_log_counter = 0;
            if (HasAnyAxisErrorCode(cache)) {
                PrintAxisErrorCodeLog(cache);
            }
        }

        if (++axis_diagnostic_log_counter >= kAxisDiagnosticLogPeriodCycles) {
            axis_diagnostic_log_counter = 0;
            PrintEnableGateDiagnosticLog(cache);
            PrintAxisDiagnosticLog(master0, cache);
            PrintAxisDiagnosticLog(master1_ctx, cache);
            PrintAxisPositionDiagnosticLog(master0, cache);
            PrintAxisPositionDiagnosticLog(master1_ctx, cache);
            PrintRuntimeSnapshot(master0, master1_ctx, offsets, cache);
        }

        SyncAndSendMaster(master0);
        SyncAndSendMaster(master1_ctx);

#ifdef MEASURE_PERF
        if (((cache.t_cur - cache.t_prev) > 1100000) || ((cache.t_cur - cache.t_prev) < 900000)) {
            time_log_flag = 1;
            printf("\nTimestamp diff: %" PRIu32 " ns\n\n", cache.t_cur - cache.t_prev);
        } else {
            time_log_flag = 0;
        }
        cache.t_prev = cache.t_cur;
#endif
    }
}

} // namespace igh_driver_internal
