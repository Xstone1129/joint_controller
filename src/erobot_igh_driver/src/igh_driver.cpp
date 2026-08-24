#include "igh_driver_internal.h"

using namespace igh_driver_internal;

/* 入口文件现在只负责组织阶段，不再承载具体实现细节。 */
int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    MasterContext master0;
    master0.master_index = 0;

    MasterContext master1_ctx;
    master1_ctx.master_index = 1;

    AxisPdoOffsetTable offsets;
    uint16_t totalSlaveCount = 0;

    if (!SetupRealtimeEnvironment()) {
        return -1;
    }

    RegisterSignalHandlers();

    if (!RequestAndProbeMasters(master0, master1_ctx, totalSlaveCount)) {
        return -1;
    }

    if (!InitializeSharedMemoryRegion()) {
        return -1;
    }

    if (!InitializeZeroDrives(master0)) {
        return -1;
    }

    if (!InitializeZeroDrives(master1_ctx)) {
        return -1;
    }

    if (!ConfigureZeroPdos(master0)) {
        return -1;
    }

    if (!ConfigureZeroPdos(master1_ctx)) {
        return -1;
    }

    const std::vector<ec_pdo_entry_reg_t> master0_regs = BuildZeroDomainRegs(master0, offsets);
    const std::vector<ec_pdo_entry_reg_t> master1_regs = BuildZeroDomainRegs(master1_ctx, offsets);

    if (!CreateAndRegisterDomain(master0, master0_regs)) {
        return -1;
    }

    if (!CreateAndRegisterDomain(master1_ctx, master1_regs)) {
        return -1;
    }

    ConfigureDistributedClocksIfEnabled(master0);
    ConfigureDistributedClocksIfEnabled(master1_ctx);

    if (!ActivateMasterAndFetchDomainData(master0)) {
        return -1;
    }

    if (!ActivateMasterAndFetchDomainData(master1_ctx)) {
        return -1;
    }

    if (!WaitUntilAllSlavesOperational(master0, master1_ctx)) {
        return -1;
    }

    RunCyclicLoop(master0, master1_ctx, offsets);
    return 0;
}
