#include "igh_driver_internal.h"

namespace igh_driver_internal {

namespace {

inline void HoldAxisPosition(uint8_t axis_num, int32_t actual_position)
{
    ec_shm_to_axis_data.axis_ctr[axis_num].axis_position = actual_position;
}

inline void MarkAxisModeBusy(uint8_t axis_num, uint8_t target_mode)
{
    ec_shm_to_axis_data.axis_ctr[axis_num].ec_mode = target_mode;
    axis_mode_states[axis_num] = AXIS_MODE_BUSY;
}

inline void MarkAxisModeSuccess(uint8_t axis_num)
{
    axis_mode_states[axis_num] = AXIS_MODE_SUCCESS;
}

inline bool IsPositionCommandInWindow(int32_t actual_position, int32_t target_position, int32_t limit)
{
    const int32_t diff = target_position - actual_position;
    return (diff > AXIS_POSITION_COMMAND_MIN_DELTA || diff < -AXIS_POSITION_COMMAND_MIN_DELTA) &&
           target_position != 0 &&
           diff < limit &&
           diff > -limit;
}

inline bool IsZeroPositionErrorWithinLimit(int32_t actual_position, int32_t target_position)
{
    const int32_t diff = target_position - actual_position;
    return diff <= ZERO_POSITION_ERROR_LIMIT && diff >= -ZERO_POSITION_ERROR_LIMIT;
}

} // namespace

/* zero 驱动模式状态机：
 * 1. 先确保模式切到目标模式。
 * 2. 模式正确后，再根据当前位置决定是否下发目标值。
 */
void axis_mode_state_chage(
    uint8_t modeword,
    uint8_t mode,
    uint8_t axis_num,
    uint16_t statusWord,
    int32_t actpos_tmp,
    int32_t targetpos_tmp,
    int32_t actvel_tmp,
    int32_t targetvel_tmp)
{
    (void)actvel_tmp;

    switch (mode) {
    case AXIS_MODE_CSP:
        if (modeword == AXIS_MODE_CSP) {
            if (statusWord != STATE_OPERATION_ENABLED) {
                HoldAxisPosition(axis_num, actpos_tmp);
            } else {
                ec_shm_to_axis_data.axis_ctr[axis_num].axis_position = targetpos_tmp;
            }
            MarkAxisModeSuccess(axis_num);
        } else {
            MarkAxisModeBusy(axis_num, CONTROL_WORD_CSP);
            HoldAxisPosition(axis_num, actpos_tmp);
        }
        break;

    case AXIS_MODE_CSV:
        if (modeword == AXIS_MODE_CSV) {
            if (statusWord == STATE_OPERATION_ENABLED) {
                ec_shm_to_axis_data.axis_ctr[axis_num].axis_velocity = targetvel_tmp;
            }
            MarkAxisModeSuccess(axis_num);
        } else {
            MarkAxisModeBusy(axis_num, CONTROL_WORD_CSV);
        }
        break;

    case AXIS_MODE_CST:
        if (modeword == AXIS_MODE_CST) {
            MarkAxisModeSuccess(axis_num);
        } else {
            MarkAxisModeBusy(axis_num, CONTROL_WORD_CST);
        }
        break;

    case AXIS_MODE_INIT:
        break;

    default:
        if (modeword == AXIS_MODE_CSP) {
            if (statusWord != STATE_OPERATION_ENABLED) {
                HoldAxisPosition(axis_num, actpos_tmp);
            } else {
                ec_shm_to_axis_data.axis_ctr[axis_num].axis_position = targetpos_tmp;
            }
            MarkAxisModeSuccess(axis_num);
        } else {
            MarkAxisModeBusy(axis_num, CONTROL_WORD_CSP);
            HoldAxisPosition(axis_num, actpos_tmp);
        }
        break;
    }
}

/* HCFA 逻辑当前没有接入主流程，保留为预留分支。 */
void axis_mode_state_chage_hcfa(
    uint8_t modeword,
    uint8_t mode,
    uint8_t axis_num,
    uint16_t statusWord,
    int32_t actpos_tmp,
    int32_t targetpos_tmp,
    int32_t actvel_tmp,
    int32_t targetvel_tmp)
{
    (void)actvel_tmp;

    switch (mode) {
    case AXIS_MODE_CSP:
        if (modeword == AXIS_MODE_CSP) {
            if (statusWord != STATE_OPERATION_ENABLED) {
                HoldAxisPosition(axis_num, actpos_tmp);
            } else if (IsPositionCommandInWindow(actpos_tmp, targetpos_tmp, HCFA_CSP_POSITION_WINDOW)) {
                ec_shm_to_axis_data.axis_ctr[axis_num].axis_position = targetpos_tmp;
            }
            MarkAxisModeSuccess(axis_num);
        } else {
            MarkAxisModeBusy(axis_num, CONTROL_WORD_CSP);
            HoldAxisPosition(axis_num, actpos_tmp);
        }
        break;

    case AXIS_MODE_CSV:
        if (modeword == AXIS_MODE_CSV) {
            if (statusWord == STATE_OPERATION_ENABLED) {
                ec_shm_to_axis_data.axis_ctr[axis_num].axis_velocity = targetvel_tmp;
            }
            MarkAxisModeSuccess(axis_num);
        } else {
            MarkAxisModeBusy(axis_num, CONTROL_WORD_CSV);
        }
        break;

    case AXIS_MODE_CST:
        printf("axis_mode_states to AXIS_MODE_CST from :%d \n", modeword);
        break;

    case AXIS_MODE_INIT:
        break;

    default:
        if (modeword == AXIS_MODE_CSP) {
            if (statusWord != STATE_OPERATION_ENABLED) {
                HoldAxisPosition(axis_num, actpos_tmp);
            } else {
                const int32_t diff = targetpos_tmp - actpos_tmp;
                if (((diff > AXIS_POSITION_COMMAND_MIN_DELTA) || (diff < -AXIS_POSITION_COMMAND_MIN_DELTA)) &&
                    targetpos_tmp != 0 &&
                    ((diff < HCFA_DEFAULT_POSITION_WINDOW) ||
                     ((actpos_tmp - targetpos_tmp) < HCFA_DEFAULT_POSITION_WINDOW))) {
                    ec_shm_to_axis_data.axis_ctr[axis_num].axis_position = targetpos_tmp;
                }
            }
            MarkAxisModeSuccess(axis_num);
        } else {
            MarkAxisModeBusy(axis_num, CONTROL_WORD_CSP);
            HoldAxisPosition(axis_num, actpos_tmp);
        }
        break;
    }
}

/* zero 驱动控制字状态机。 */
void axis_ctrword_state_chage(uint8_t ctrstate, uint8_t axis_num)
{
    switch (ctrstate) {
    case STATE_FAULT:
        ec_shm_to_axis_data.axis_ctr[axis_num].ec_ctrword = CONTROL_WORD_FAULT_RESET;
        break;

    case STATE_SWITCH_ON_DISABLED:
        ec_shm_to_axis_data.axis_ctr[axis_num].ec_ctrword = CONTROL_WORD_SHUTDOWN;
        break;

    case STATE_READY_TO_SWITCH_ON:
        ec_shm_to_axis_data.axis_ctr[axis_num].ec_ctrword = CONTROL_WORD_SWITCH_ON;
        break;

    case STATE_SWITCHED_ON:
        ec_shm_to_axis_data.axis_ctr[axis_num].ec_ctrword = CONTROL_WORD_ENABLE_OPERATION;
        break;

    case STATE_OPERATION_ENABLED:
        ec_shm_to_axis_data.axis_ctr[axis_num].ec_ctrword = CONTROL_WORD_ENABLE_OPERATION;
        break;

    default:
        ec_shm_to_axis_data.axis_ctr[axis_num].ec_ctrword = CONTROL_WORD_SHUTDOWN;
        break;
    }
}

/* 预留给 Hechuan 驱动的控制字状态机。 */
void hechuan_ctrword_state_chage(uint8_t ctrstate, uint8_t axis_num)
{
    switch (ctrstate) {
    case STATE_FAULT:
    case STATE_FAULT_HECHUAN:
    case STATE_FAULT_HECHUAN1:
        ec_shm_to_axis_data.axis_ctr[axis_num].ec_ctrword = CONTROL_WORD_FAULT_RESET;
        break;

    case STATE_SWITCH_ON_DISABLED:
        ec_shm_to_axis_data.axis_ctr[axis_num].ec_ctrword = CONTROL_WORD_SHUTDOWN;
        break;

    case STATE_READY_TO_SWITCH_ON:
        ec_shm_to_axis_data.axis_ctr[axis_num].ec_ctrword = CONTROL_WORD_SWITCH_ON;
        break;

    case STATE_SWITCHED_ON:
    case STATE_OPERATION_ENABLED:
        ec_shm_to_axis_data.axis_ctr[axis_num].ec_ctrword = CONTROL_WORD_ENABLE_OPERATION;
        break;

    default:
        ec_shm_to_axis_data.axis_ctr[axis_num].ec_ctrword = CONTROL_WORD_SHUTDOWN;
        break;
    }
}

/* 预留给 PXB 驱动的控制字状态机。 */
void pxb_ctrword_state_chage(uint8_t ctrstate, uint8_t axis_num)
{
    switch (ctrstate) {
    case PXBSTATE_FAULT:
    case PXBSTATE_FAULT1:
        ec_shm_to_axis_data.axis_ctr[axis_num + 17].ec_ctrword = CONTROL_WORD_FAULT_RESET;
        break;

    case PXBSTATE_SWITCH_ON_DISABLED1:
    case STATE_SWITCH_ON_DISABLED:
        ec_shm_to_axis_data.axis_ctr[axis_num + 17].ec_ctrword = CONTROL_WORD_SHUTDOWN;
        break;

    case PXBSTATE_READY_TO_SWITCH_ON:
        ec_shm_to_axis_data.axis_ctr[axis_num + 17].ec_ctrword = CONTROL_WORD_SWITCH_ON;
        break;

    case PXBSTATE_SWITCHED_ON:
    case PXBSTATE_OPERATION_ENABLED:
        ec_shm_to_axis_data.axis_ctr[axis_num + 17].ec_ctrword = CONTROL_WORD_ENABLE_OPERATION;
        break;

    default:
        ec_shm_to_axis_data.axis_ctr[axis_num + 17].ec_ctrword = CONTROL_WORD_SHUTDOWN;
        break;
    }
}

} // namespace igh_driver_internal
