# 升降控制器故障日志与分级诊断说明

> 适用范围：下位机 `/home/user/joint_controller` 的 `joint_hardware` 包内
> `LiftController`（`src/joint_hardware/src/lift/lift_controller.cpp`）。
>
> 本文回答两个问题：**故障为什么曾经查不到日志**，以及**现在怎么查、去哪查**。

## 1. 问题背景：4 个 `Mode::fault` 入口曾经完全静默

改动前，`LiftController` 进入 `Mode::fault` 的位置共有 4 处，全部只写
`status_message_` 字符串（发给 `/joint/lift/control_status` 的 JSON），
**没有任何 `RCLCPP_*` 调用**，因此会话日志文件里永远不会有故障记录：

| 故障码 | 触发条件 | 说明 |
| --- | --- | --- |
| `non_finite_state_interfaces` | 状态接口读到 NaN/Inf | `update()` 直接返回 `ERROR` |
| `invalid_jog_velocity` | Jog 速度目标设置失败 | Ruckig 拒绝该速度/比例 |
| `gate_lost_during_motion` | 运动中 `driver_gate_ready()` 失败 | CiA 402/PDO 抖动即可触发 |
| `ruckig_rejected_state` | Ruckig 轮廓 `valid=false` | 运动状态被拒 |

后果：驱动器 PDO 微秒级抖动 → 门控瞬时失效 → 控制器锁入 `fault` → 任务中止，
而日志里**什么都看不到**，只能靠重启运行时才能清除。

## 2. 现在的日志分类与等级

所有诊断行都带稳定的分类标签，便于对**同一个会话日志**做过滤：

| 标签 | 含义 | 等级 | 节流 |
| --- | --- | --- | --- |
| `[LIFT_FAULT]` | 故障锁定 / 锁存被清除 | `ERROR` / `WARN` | 同一故障码 1 行/秒 |
| `[LIFT_GATE]` | 门控降级 / 恢复 | `DEBUG`（持续 2 s 后升级 `WARN`，恢复时 `INFO`） | `DEBUG` 1 行/秒，`WARN` 1 行/5 s |
| `[LIFT_CMD]` | 命令被拒绝或忽略 | `DEBUG` | 10 行/秒上限 |
| `[LIFT_CFG]` | 配置与生命周期 | `INFO` | 每次配置 1 行 |

关键设计点：

- **故障一定可见**：`record_lift_fault()` 负责输出 `ERROR`，锁存期间每秒一次心跳，
  并带上 `total`（累计次数）、`repeats_burst`（本次连发次数）、
  `suppressed_since_last`（上次输出后被抑制的控制周期数）。
- **详情里带完整门控分解**：`gate[...]` 字段由 `driver_gate_failure_reason()`
  生成，一次就能看出是 `feedback_fresh`、`ethercat_operational`、
  `working_counter_ok`、`power_enabled_state`、`cia402_operation_enabled`、
  `estop_latched`、`driver_error_code`、`mode_display` 中哪一项失败。
- **不污染实时循环**：故障锁存期间分支每 100 Hz 重入，因此详情字符串**只在真正
  输出日志时才构造**（`record_lift_fault` 的模板参数是一个惰性 lambda），
  其余周期只走一个无分配的计数路径。
- **历史不被冲掉**：故障历史是 8 条环形缓冲，同一次连发（60 s 窗口内相同故障码）
  只占 1 条记录，用 `repeats` 计数，不再把之前不同的故障挤出历史。

## 3. 日志落在哪里（同一会话）

下位机所有入口都 `source scripts/runtime_log_env.sh`，它会：

1. 读取上位机写下的会话名 `.junior_runtime_session_name`；
2. 把 `ROS_LOG_DIR` 指向 `${HOME}/.ros/log/<会话名>/ros`；
3. 把 `RUNTIME_LOG_DIR` 指向 `${HOME}/.ros/log/<会话名>/runtime`。

所以**上位机和下位机的同一次运行共用同一个会话目录**，升降控制器运行在
`ros2_control_node` 进程内，其日志出现在：

```text
/home/user/.ros/log/<会话名>/ros/ros2_control_node_<pid>_<时间戳>.log
```

查看本次会话的故障记录：

```bash
# 最近一次会话
SESSION=$(ls -1dt /home/user/.ros/log/*/ | head -1)

# 所有故障锁定记录
grep '\[LIFT_FAULT\]' "$SESSION"ros/ros2_control_node_*.log

# 门控降级/恢复过程
grep '\[LIFT_GATE\]' "$SESSION"ros/ros2_control_node_*.log

# 被拒绝的命令与配置
grep -E '\[LIFT_CMD\]|\[LIFT_CFG\]' "$SESSION"ros/ros2_control_node_*.log
```

`DEBUG` 级需要显式打开（默认只输出 `INFO` 及以上）：

```bash
ros2 run ... --ros-args --log-level lift_controller:=debug
# 或对运行中的进程：
ros2 service call /lift/controller_manager/set_logger_levels ...
```

## 4. 状态话题中的诊断字段

`/joint/lift/control_status`（`std_msgs/String`，JSON）新增：

| 字段 | 含义 |
| --- | --- |
| `gate_ready` | 门控当前是否就绪 |
| `gate_failure` | 未就绪时列出**每一项**失败的子条件及其数值，就绪时为 `none` |
| `fault_total` | 本次激活以来的故障累计次数 |
| `fault_history[]` | 最近 8 条故障：`age_ms`（首次）、`last_age_ms`（最近）、`repeats`、`code`、`detail` |

于是即使故障已经恢复（或门控只是降级），用一条 `ros2 topic echo` 也能看出
"刚才到底发生了什么"，不必等下一次复现。

## 5. 单元测试

`src/joint_hardware/test/test_lift_controller.cpp` 中新增 5 个测试，测试环境自带
隔离（`ROS_DOMAIN_ID=155`、`ROS_LOCALHOST_ONLY=1`），不会影响现场运行：

| 测试 | 验证内容 |
| --- | --- |
| `GateLossDuringMotionLatchesFaultAndCountsRepeats` | 运动中门控丢失 → 锁存 `fault`，状态 JSON 含故障码与门控分解，历史只占 1 条且 `repeats == fault_total` |
| `LatchedFaultSurvivesGateRecoveryUntilReactivation` | 门控恢复后**锁存仍在**、计数冻结；重新激活才清除并输出 `state=cleared_by_activate` |
| `LatchedFaultEmitsCategorizedErrorLine` | 通过 rcutils 日志回调断言故障确实以 `ERROR` 输出，分类标签为 `[LIFT_FAULT]` |
| `DegradedGateIsVisibleWithoutLatchingAFault` | 门控降级但未运动时不锁存故障，`DEBUG`/`WARN`/`INFO` 三级都出现 |
| `IdleUnpoweredHoldDoesNotFaultOrWarn` | 未上电静止 HOLD 不产生 `WARN` 噪声（门控"未就绪"是设计状态） |

运行方式：

```bash
cd /home/user/joint_controller
source /opt/ros/humble/setup.bash && source install/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=12
colcon build --symlink-install --parallel-workers 12 --packages-select joint_hardware
colcon test --packages-select joint_hardware --ctest-args -R test_lift_controller \
  --event-handlers console_direct+
```

## 6. 未改动项（重要）

本次**没有**改变故障锁存语义：进入 `Mode::fault` 后仍然不会自动恢复，
只能通过重新激活控制器（重启运行时）清除。自动恢复属于安全行为变更，
需要显式评审后再做，本次仅补齐可观测性。

## 7. 验收记录

- 2026-09-11：下位机 `colcon build --symlink-install --parallel-workers 12
  --packages-select joint_hardware` 通过；`test_lift_controller` 全部用例通过
  （含新增 5 例），运行时间约 15 s。
- 现场 REAL 栈（`joint-controller-stack-real`）在测试期间保持运行，测试使用
  隔离 ROS 域，未向现场总线域发布任何命令，未触发任何运动。
