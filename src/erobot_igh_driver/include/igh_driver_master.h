#ifndef __IGH_DRIVER_MASTER_H__
#define __IGH_DRIVER_MASTER_H__

/*****************************************************************************/
#include <ecrt.h> 
#include <string.h>
#include <stdio.h>
/* For setting the process's priority (setpriority) */
#include <sys/resource.h>
/* For pid_t and getpid() */
#include <unistd.h>
#include <sys/types.h>
/* For locking the program in RAM (mlockall) to prevent swapping */

#include <sys/mman.h>
/* clock_gettime, struct timespec, etc. */
#include <time.h>
/* Header for handling signals (definition of SIGINT) */
#include <signal.h>
/* For using real-time scheduling policy (FIFO) and sched_setaffinity */
#include <sched.h>
/* For using uint32_t format specifier, PRIu32 */
#include <inttypes.h>

#include <iostream>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <memory>

/* Comment to disable PDO configuration (i.e. in case the PDO configuration saved in EEPROM is our
   desired configuration.)
*/
#define CONFIG_PDOS

/* Comment to disable distributed clocks. */
#define DC

/* Choose the syncronization method: The reference clock can be either master's, or the reference slave's (slave 0 by default) */
#ifdef DC

/* Slave0's clock is the reference: no drift. Algorithm from rtai_rtdm_dc example. Work in progress. */
//#define SYNC_MASTER_TO_REF
/* Master's clock (CPU) is the reference: lower overhead. */
#define SYNC_REF_TO_MASTER

#endif

#ifdef DC

/* Comment to disable configuring slave's DC specification (shift time & cycle time) */
#define CONFIG_DC

#endif

/*****************************************************************************/

/* One motor revolution increments the encoder by 2^19 -1. */
#define ENCODER_RES 524287
/* The maximum stack size which is guranteed safe to access without faulting. */
#define MAX_SAFE_STACK (8 * 1024)

/* Uncomment to enable performance measurement. */
/* Measure the difference in reference slave's clock timstamp each cycle, and print the result,
   which should be as close to cycleTime as possible. */
/* Note: Only works with DC enabled. */
#define MEASURE_PERF

/* Calculate the time it took to complete the loop. */
#define MEASURE_TIMING

#define SET_CPU_AFFINITY

#define PXBNSEC_PER_SEC (2000000000L)
#define NSEC_PER_SEC (1000000000L)
#define FREQUENCY 1000
/* Period of motion loop, in nanoseconds */
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)
#define PXBPERIOD_NS (PXBNSEC_PER_SEC / FREQUENCY)

#ifdef DC

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

#endif

#ifdef CONFIG_DC

/* SYNC0 event happens halfway through the cycle */
#define SHIFT0 (PERIOD_NS/2)

#endif
/*****************************************************************************/
#define zero_vendor_id_err 0x5a6572
#define zero_vendor_id 0x5a65726f
#define zero_product_code 0x00029252
#define eyou_vendor_id 0x00001097
#define eyou_product_code 0x00002406
#define hcfa_vendor_id 0x000116c7
#define hcfa_product_code 0x003e0402

#define pxb_vendor_id 0x00201811
#define pxb_product_code 0x01050001

//uint8_t pxb_vendor_runtimer = 0;
//uint8_t pxb_vendor_runtimer_read = 0;
/* Note: Anything relying on definition of SYNC_MASTER_TO_REF is essentially copy-pasted from /rtdm_rtai_dc/main.c */

#ifdef SYNC_MASTER_TO_REF

/* First used in system_time_ns() */
static int64_t  system_time_base = 0LL;
/* First used in sync_distributed_clocks() */
static uint64_t dc_time_ns = 0;
static int32_t  prev_dc_diff_ns = 0;
/* First used in update_master_clock() */
static int32_t  dc_diff_ns = 0;
static unsigned int cycle_ns = PERIOD_NS;
static uint8_t  dc_started = 0;
static int64_t  dc_diff_total_ns = 0LL;
static int64_t  dc_delta_total_ns = 0LL;
static int      dc_filter_idx = 0;
static int64_t  dc_adjust_ns;
#define DC_FILTER_CNT          1024
/** Return the sign of a number
 *
 * ie -1 for -ve value, 0 for 0, +1 for +ve value
 *
 * \retval the sign of the value
 */
#define sign(val) \
    ({ typeof (val) _val = (val); \
    ((_val > 0) - (_val < 0)); })

static uint64_t dc_start_time_ns = 0LL;

#endif
/*****************************************************************************/
#define EC_NEWTIMEVAL2NANO(TV) \
(((TV).tv_sec - 946684800ULL) * 1000000000ULL + (TV).tv_nsec)
//uint32_t interval_=(uint32_t)(1000000000.0 / 1000);

// Add state definitions
#define STATE_FAULT              0x0008
#define STATE_SWITCH_ON_DISABLED 0x0040
#define STATE_READY_TO_SWITCH_ON 0x0021
#define STATE_SWITCHED_ON        0x0023
#define STATE_OPERATION_ENABLED  0x0027

#define STATE_FAULT_HECHUAN              0x002B
#define STATE_FAULT_HECHUAN1              0x000F

#define PXBSTATE_FAULT              0x0038
#define PXBSTATE_FAULT1              0x0018
#define PXBSTATE_SWITCH_ON_DISABLED1              0x0070
#define PXBSTATE_SWITCH_ON_DISABLED 0x0040
#define PXBSTATE_READY_TO_SWITCH_ON 0x0031
#define PXBSTATE_SWITCHED_ON        0x0033
#define PXBSTATE_OPERATION_ENABLED  0x0037

// Add these control word commands
#define CONTROL_WORD_SHUTDOWN           0x0006
#define CONTROL_WORD_SWITCH_ON         0x0007
#define CONTROL_WORD_ENABLE_OPERATION  0x000F
#define CONTROL_WORD_FAULT_RESET       0x0080

// Add these mode control word commands
#define CONTROL_WORD_CSP            0x0008
#define CONTROL_WORD_CSV         	0x0009
#define CONTROL_WORD_CST  			0x000A

// Add these mode control word commands
#define AXIS_MODE_CSP           0x0008
#define AXIS_MODE_CSV         	0x0009
#define AXIS_MODE_CST  			0x000A
#define AXIS_MODE_INIT			0x0000
#define AXIS_MODE_BUSY  		0x0001
#define AXIS_MODE_SUCCESS    	0x0002

#define AXIS_NUM  			19
#define AXIS_STATUES_ERR  			0x08
#define AXIS_POSITION_COMMAND_MIN_DELTA 5
#define CSP_REENTRY_COMMAND_WINDOW 500
#define ZERO_POSITION_ERROR_LIMIT 300000
#define HCFA_CSP_POSITION_WINDOW 5000
#define HCFA_DEFAULT_POSITION_WINDOW 100


/*****************************************************************************/
/** Application-ethercat axis share_mem.
 */
 /** send axis data to APP form axis.*/
typedef struct {
    uint8_t ec_modestate; /**< Slave alias address. */
    uint8_t ec_ctrstate; /**< Slave position. */
    int32_t axis_position ; /**< Slave vendor ID. */
    int32_t axis_velocity; /**< Slave product code. */
    int32_t axis_effort; /**< PDO entry index. */
    int32_t axis_encoder_0; /**< Slave product code. */ 
    int32_t axis_encoder_1; /**< Slave product code. */ 
    uint16_t axis_error_code; /**< Latest 0x603F fault code. */
    uint8_t axis_counter; /**< PDO entry subindex. */
    uint8_t axis_checksum; /**< PDO entry subindex. */
} ec_app_axis_write_reg_t;

 /** send axis data to axis form APP.*/
typedef struct {
    uint8_t ec_mode; /**< Slave alias address. */
    uint8_t ec_ctrword; /**< Slave position. */
    int32_t axis_position ; /**< Slave vendor ID. */
    int32_t axis_velocity; /**< Slave product code. */
    int32_t axis_effort; /**< PDO entry index. */
    uint8_t axis_counter; /**< PDO entry subindex. */
    uint8_t axis_checksum; /**< PDO entry subindex. */
} ec_app_axis_read_reg_t;

/** send axis data to APP form axis.*/
typedef struct {
    uint8_t ec_powerstate; /**< Slave alias address. */
    ec_app_axis_write_reg_t axis_state[23]; /**< PDO entry subindex. */
} ec_app_write_reg_t;

/** send axis data to axis form APP.*/
typedef struct {
    uint8_t ec_poweron; /**< Slave alias address. */
    ec_app_axis_read_reg_t axis_ctr[23]; /**< PDO entry subindex. */
    uint8_t ec_axis_io[16];
} ec_app_read_reg_t;
/*****************************************************************************/


#endif
