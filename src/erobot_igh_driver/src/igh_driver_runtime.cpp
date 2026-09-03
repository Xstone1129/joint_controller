#include "igh_driver_internal.h"

#include <cstring>
#include <exception>
#include <memory>

namespace igh_driver_internal {

/* 共享内存运行时镜像。 */
ec_app_read_reg_t ec_shm_desire_axis_data{};
ec_app_read_reg_t ec_shm_to_axis_data{};
ec_app_read_reg_t* ec_shm_desire_axis_data_ptr = nullptr;

ec_app_write_reg_t ec_shm_real_axis_data{};
ec_app_write_reg_t* ec_shm_real_axis_data_ptr = nullptr;
SharedMotorMonitorDebug* shared_motor_monitor_debug_ptr = nullptr;

/* 双主站句柄保留为全局，信号处理时可以统一释放。 */
ec_master_t* master = nullptr;
ec_master_t* master1 = nullptr;

/* 运行期轴状态缓存。 */
std::array<uint8_t, kAxisStateSlotCount> axis_mode_states{};
std::array<uint8_t, kAxisStateSlotCount> axis_counter_error{};
std::array<DriveRuntimeProfile, kMaxZeroAxisCount> axis_drive_profiles{};

namespace {

std::shared_ptr<boost::interprocess::shared_memory_object> shm_desire_axis_ptr = nullptr;
std::shared_ptr<boost::interprocess::mapped_region> mapped_region_desire_axis_ptr = nullptr;
std::shared_ptr<boost::interprocess::shared_memory_object> shm_real_axis_ptr = nullptr;
std::shared_ptr<boost::interprocess::mapped_region> mapped_region_real_axis_ptr = nullptr;
std::shared_ptr<boost::interprocess::shared_memory_object> shm_debug_ptr = nullptr;
std::shared_ptr<boost::interprocess::mapped_region> mapped_region_debug_ptr = nullptr;

template <typename T>
bool CreateSharedMemoryBlock(
    const char* name,
    std::shared_ptr<boost::interprocess::shared_memory_object>& shm_ptr,
    std::shared_ptr<boost::interprocess::mapped_region>& region_ptr,
    T*& typed_ptr,
    bool clear_on_attach)
{
    boost::interprocess::permissions perm;
    perm.set_unrestricted();

    bool created = false;
    try {
        shm_ptr = std::make_shared<boost::interprocess::shared_memory_object>(
            boost::interprocess::open_only,
            name,
            boost::interprocess::read_write);
    } catch (const boost::interprocess::interprocess_exception&) {
        shm_ptr = std::make_shared<boost::interprocess::shared_memory_object>(
            boost::interprocess::create_only,
            name,
            boost::interprocess::read_write,
            perm);
        created = true;
    }

    boost::interprocess::offset_t current_size = 0;
    shm_ptr->get_size(current_size);
    if (created || current_size < static_cast<boost::interprocess::offset_t>(sizeof(T))) {
        shm_ptr->truncate(sizeof(T));
    }

    region_ptr = std::make_shared<boost::interprocess::mapped_region>(
        *shm_ptr,
        boost::interprocess::read_write);

    typed_ptr = static_cast<T*>(region_ptr->get_address());
    if (created || clear_on_attach) {
        std::memset(typed_ptr, 0, sizeof(T));
    }
    return true;
}

/* mlockall 只会锁定已经分配的页，所以要先触碰一遍安全栈。 */
void stack_prefault()
{
    unsigned char dummy[MAX_SAFE_STACK];
    std::memset(dummy, 0, MAX_SAFE_STACK);
}

} // namespace

/* 时间加法用于构造绝对唤醒时间。 */
void timespec_add(struct timespec* result, struct timespec* time1, struct timespec* time2)
{
    if ((time1->tv_nsec + time2->tv_nsec) >= NSEC_PER_SEC) {
        result->tv_sec = time1->tv_sec + time2->tv_sec + 1;
        result->tv_nsec = time1->tv_nsec + time2->tv_nsec - NSEC_PER_SEC;
    } else {
        result->tv_sec = time1->tv_sec + time2->tv_sec;
        result->tv_nsec = time1->tv_nsec + time2->tv_nsec;
    }
}

#ifdef MEASURE_TIMING
/* 时间减法只用于周期抖动测量。 */
void timespec_sub(struct timespec* result, struct timespec* time1, struct timespec* time2)
{
    if ((time1->tv_nsec - time2->tv_nsec) < 0) {
        result->tv_sec = time1->tv_sec - time2->tv_sec - 1;
        result->tv_nsec = NSEC_PER_SEC - (time2->tv_nsec - time1->tv_nsec);
    } else {
        result->tv_sec = time1->tv_sec - time2->tv_sec;
        result->tv_nsec = time1->tv_nsec - time2->tv_nsec;
    }
}
#endif

#ifdef SYNC_MASTER_TO_REF
/* 针对每个 master 独立维护 DC 同步基准。 */
uint64_t system_time_ns(DcSyncContext& dc_sync)
{
    struct timespec time;
    int64_t time_ns;
    clock_gettime(CLOCK_MONOTONIC, &time);
    time_ns = TIMESPEC2NS(time);

    if (dc_sync.system_time_base > time_ns) {
        printf("%s() error: system_time_base greater than system time (system_time_base: %ld, time: %ld)\n",
               __func__,
               dc_sync.system_time_base,
               time_ns);
        return time_ns;
    }

    return time_ns - dc_sync.system_time_base;
}

void sync_distributed_clocks(MasterContext& ctx)
{
    if (ctx.handle == nullptr) {
        return;
    }

    uint32_t ref_time = 0;
    uint64_t prev_app_time = ctx.dc_sync.dc_time_ns;

    ctx.dc_sync.dc_time_ns = system_time_ns(ctx.dc_sync);
    ecrt_master_application_time(ctx.handle, ctx.dc_sync.dc_time_ns);
    ecrt_master_reference_clock_time(ctx.handle, &ref_time);
    ctx.dc_sync.dc_diff_ns = static_cast<uint32_t>(prev_app_time) - ref_time;
    ecrt_master_sync_slave_clocks(ctx.handle);
}

void update_master_clock(MasterContext& ctx)
{
    int32_t delta = ctx.dc_sync.dc_diff_ns - ctx.dc_sync.prev_dc_diff_ns;
    ctx.dc_sync.prev_dc_diff_ns = ctx.dc_sync.dc_diff_ns;

    ctx.dc_sync.dc_diff_ns =
        ((ctx.dc_sync.dc_diff_ns + (ctx.dc_sync.cycle_ns / 2)) % ctx.dc_sync.cycle_ns) -
        (ctx.dc_sync.cycle_ns / 2);

    if (ctx.dc_sync.dc_started) {
        ctx.dc_sync.dc_diff_total_ns += ctx.dc_sync.dc_diff_ns;
        ctx.dc_sync.dc_delta_total_ns += delta;
        ctx.dc_sync.dc_filter_idx++;

        if (ctx.dc_sync.dc_filter_idx >= DC_FILTER_CNT) {
            ctx.dc_sync.dc_adjust_ns +=
                ((ctx.dc_sync.dc_delta_total_ns + (DC_FILTER_CNT / 2)) / DC_FILTER_CNT);
            ctx.dc_sync.dc_adjust_ns += sign(ctx.dc_sync.dc_diff_total_ns / DC_FILTER_CNT);

            if (ctx.dc_sync.dc_adjust_ns < -1000) {
                ctx.dc_sync.dc_adjust_ns = -1000;
            }
            if (ctx.dc_sync.dc_adjust_ns > 1000) {
                ctx.dc_sync.dc_adjust_ns = 1000;
            }

            ctx.dc_sync.dc_diff_total_ns = 0LL;
            ctx.dc_sync.dc_delta_total_ns = 0LL;
            ctx.dc_sync.dc_filter_idx = 0;
        }

        ctx.dc_sync.system_time_base += ctx.dc_sync.dc_adjust_ns + sign(ctx.dc_sync.dc_diff_ns);
    } else {
        ctx.dc_sync.dc_started = (ctx.dc_sync.dc_diff_ns != 0);

        if (ctx.dc_sync.dc_started) {
            printf("%s first master diff: %d.\n", GetMasterName(ctx), ctx.dc_sync.dc_diff_ns);
            ctx.dc_sync.dc_start_time_ns = ctx.dc_sync.dc_time_ns;
        }
    }
}
#endif

void ReleaseMasterIfNeeded(ec_master_t*& current_master)
{
    if (current_master != nullptr) {
        ecrt_release_master(current_master);
        current_master = nullptr;
    }
}

/* 一个统一的信号处理函数，避免两个 master 互相覆盖注册。 */
void ReleaseAllMastersAndExit(int sig)
{
    (void)sig;
    printf("\nReleasing masters...\n");
    ReleaseMasterIfNeeded(master);
    ReleaseMasterIfNeeded(master1);
    kill(getpid(), SIGKILL);
}

void RegisterSignalHandlers()
{
    signal(SIGTERM, ReleaseAllMastersAndExit);
    signal(SIGINT, ReleaseAllMastersAndExit);
}

/* 只负责进入实时环境，不混合 EtherCAT 初始化。 */
bool SetupRealtimeEnvironment()
{
#ifdef SET_CPU_AFFINITY
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(15, &set);

    if (sched_setaffinity(0, sizeof(set), &set)) {
        printf("Setting CPU affinity failed!\n");
        return false;
    }
#endif

    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    printf("Using priority %i.\n", param.sched_priority);

    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed\n");
    }

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        printf("mlockall failed\n");
        return false;
    }

    stack_prefault();
    return true;
}

/* 初始化 APP 与 EtherCAT 线程共享的两块共享内存。 */
bool InitializeSharedMemoryRegion()
{
    try {
        CreateSharedMemoryBlock(
            "Ethercat_axis_desire",
            shm_desire_axis_ptr,
            mapped_region_desire_axis_ptr,
            ec_shm_desire_axis_data_ptr,
            false);
        CreateSharedMemoryBlock(
            "Ethercat_axis_real",
            shm_real_axis_ptr,
            mapped_region_real_axis_ptr,
            ec_shm_real_axis_data_ptr,
            true);
        CreateSharedMemoryBlock(
            "Ethercat_axis_debug",
            shm_debug_ptr,
            mapped_region_debug_ptr,
            shared_motor_monitor_debug_ptr,
            true);

        std::memcpy(&ec_shm_desire_axis_data, ec_shm_desire_axis_data_ptr, sizeof(ec_shm_desire_axis_data));
        ec_shm_desire_axis_data.ec_poweron = 0;
        std::memcpy(ec_shm_desire_axis_data_ptr, &ec_shm_desire_axis_data, sizeof(ec_shm_desire_axis_data));
        std::memset(&ec_shm_to_axis_data, 0, sizeof(ec_shm_to_axis_data));
        std::memset(&ec_shm_real_axis_data, 0, sizeof(ec_shm_real_axis_data));
        if (shared_motor_monitor_debug_ptr != nullptr) {
            shared_motor_monitor_debug_ptr->magic = kSharedMotorMonitorDebugMagic;
        }
        std::memset(axis_counter_error.data(), 0, axis_counter_error.size());

        return true;
    } catch (const std::exception& ex) {
        printf("Initialize shared memory failed: %s\n", ex.what());
        return false;
    }
}

} // namespace igh_driver_internal
