#ifndef __IGH_DRIVER_CR_H__
#define __IGH_DRIVER_CR_H__

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

extern void ODwrite(ec_master_t* master, uint16_t slavePos, uint16_t index, uint8_t subIndex, uint8_t objectValue);
extern void ODwrite_u16(ec_master_t* master, uint16_t slavePos, uint16_t index, uint8_t subIndex, uint16_t objectValue);
extern void ODwrite_u32(ec_master_t* master, uint16_t slavePos, uint16_t index, uint8_t subIndex, uint32_t objectValue);
extern void ODwrite_607F(ec_master_t* master, uint16_t slavePos, uint16_t index, uint8_t subIndex);
extern void ODwrite_6080(ec_master_t* master, uint16_t slavePos, uint16_t index, uint8_t subIndex);
extern void ODwrite_6065(ec_master_t* master, uint16_t slavePos, uint16_t index, uint8_t subIndex);
extern void initDrive_zero(ec_master_t* master, uint16_t slavePos , uint8_t axismode, uint16_t feedforward_gain);
extern void initDrive_eyou_basic(ec_master_t* master, uint16_t slavePos, uint8_t axismode);
extern void initDrive_hcfa(ec_master_t* master, uint16_t slavePos , uint8_t axismode);
extern void init_chage_Drive(ec_master_t* master, uint16_t slavePos , uint8_t axismode);

#endif
