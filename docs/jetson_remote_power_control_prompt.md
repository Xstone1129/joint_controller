# Jetson 端远程电机使能/失能修改提示词

请修改 Jetson 工作空间 `~/joint_controller`，在保留现有 workspace 控制、遥测订阅和选择性关节控制功能的基础上，增加机械臂 14 个电机的远程使能/失能闭环控制。

Ubuntu 端已经完成以下实现并在仿真中验证通过：

- 服务：`/set_robot_power`
- 服务类型：`robot_control_msg/srv/SetRobotPower`
- 底层命令话题：`/robot_poweron`，只能由 Ubuntu 的 `/set_robot_power` 服务发布
- 状态话题：`/arm/power_status`
- 状态类型：`robot_control_msg/msg/ArmPowerStatus`
- `status == 39` 表示使能，`status == 64` 表示失能
- Ubuntu 服务会等待 14 个电机的实际反馈，达到目标后才返回 `success=true`

## 一、同步接口

在 `robot_control_msg/msg/ArmPowerStatus.msg` 新增或确认以下完全一致的内容：

```text
int32 ENABLED_STATUS=39
int32 DISABLED_STATUS=64

builtin_interfaces/Time stamp

string[] joint_names
int32[] status_codes
bool[] enabled

bool command_enabled
bool all_enabled

string message
```

确认 `robot_control_msg/srv/SetRobotPower.srv` 与 Ubuntu 完全一致：

```text
bool enable
---
bool success
string message
```

把 `ArmPowerStatus.msg` 和 `SetRobotPower.srv` 加入 `robot_control_msg/CMakeLists.txt` 的 `rosidl_generate_interfaces()`。保留其他已有接口。

## 二、修改 planner_node_jetson

在 `planner_node_jetson.cpp` 中增加：

1. `/arm/power_status` subscription，类型为 `robot_control_msg::msg::ArmPowerStatus`。
2. Ubuntu `/set_robot_power` client，类型为 `robot_control_msg::srv::SetRobotPower`。
3. Jetson 对外服务 `/planner/set_robot_power`，类型同样为 `robot_control_msg::srv::SetRobotPower`。

Jetson 不得直接发布 `/robot_poweron`，所有电机使能/失能命令必须通过 Ubuntu `/set_robot_power` 服务执行。

`/arm/power_status` QoS 必须为：

```cpp
rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local()
```

保存最新状态、接收时间和消息计数，使用 mutex 保护。状态必须校验：

- `joint_names.size() == 14`
- `status_codes.size() == 14`
- `enabled.size() == 14`
- `enabled[i]` 必须与 `status_codes[i] == ArmPowerStatus::ENABLED_STATUS` 一致
- `all_enabled` 必须与 14 个 `enabled` 的逻辑与一致

## 三、安全参数

新增参数：

```text
subscribe_power_status=true
power_status_stale_timeout_ms=1000
power_service_timeout_ms=7000
enable_remote_power_commands=false
enable_remote_real_power_commands=false
```

把这些参数加入 `planner_node_jetson.launch.py` 并传给节点。

`enable_remote_power_commands` 是所有远程使能命令的总开关，默认必须为 `false`。

`enable_remote_real_power_commands` 是真机使能的第二道开关，默认必须为 `false`。真机使能必须同时显式打开这两个参数。

失能命令属于安全动作：即使上述参数为 `false`、workspace 不为 RUNNING、状态不可用或过期，也应尝试调用 Ubuntu `/set_robot_power` 执行失能；但仍要把 Ubuntu 的真实结果返回给调用方，不能伪造成功。

## 四、使能请求门控

收到 `/planner/set_robot_power` 的 `enable=true` 时，按以下顺序检查，任一失败都不调用 Ubuntu 服务：

1. `enable_remote_power_commands == true`
2. 最新 `/workspace/status` 为 `accepted=true` 且 `state=RUNNING`
3. 已收到结构合法且未超过 `power_status_stale_timeout_ms` 的 `/arm/power_status`
4. 当前 workspace mode 是 `SIMULATION` 或 `REAL`，不能是 `UNKNOWN`
5. 如果 mode 为 `REAL`，还必须满足 `enable_remote_real_power_commands == true`
6. 同一时刻没有其他 power 请求正在执行

建议的拒绝消息：

```text
remote power commands are disabled; launch with enable_remote_power_commands:=true
workspace is not RUNNING
/arm/power_status is unavailable, stale, or malformed
workspace mode is UNKNOWN
real power commands are disabled; launch with enable_remote_real_power_commands:=true
another power command is already in progress
```

如果请求目标已经由最新实际状态满足，允许直接返回成功：

- enable：`command_enabled=true`、`all_enabled=true`、14/14 `enabled=true`
- disable：`command_enabled=false`、14/14 `enabled=false`

## 五、闭环返回语义

调用 Ubuntu `/set_robot_power` 后必须等待最终响应，不能只返回“submitted”。

- Ubuntu 返回 `success=false`：Jetson 原样返回 `success=false`，并在 message 中保留 Ubuntu 错误原因。
- Ubuntu 返回 `success=true`：再确认 Jetson 收到命令发出后的新 `/arm/power_status`，且状态达到目标；然后才向调用方返回 `success=true`。
- 等待服务或响应超时：返回 `success=false`。
- Ubuntu 成功但 Jetson 在超时内未观察到匹配状态：返回 `success=false`，说明下游服务回执与 Jetson 遥测未完成交叉确认。

成功消息应明确模式与实际数量，例如：

```text
SIMULATION power enabled and confirmed: 14/14 motors status 39
SIMULATION power disabled and confirmed: 0/14 motors enabled
REAL power enabled and confirmed: 14/14 motors status 39
REAL power disabled and confirmed: 0/14 motors enabled
```

不要把 WorkspaceControl 的“已发布”语义套用到电机服务。`/planner/set_robot_power` 的 `success=true` 必须表示最终状态已确认。

## 六、避免服务死锁

`/planner/set_robot_power` 服务回调需要等待 Ubuntu client 的响应。必须使用独立 callback group，并把节点主函数改为至少 2 线程的 `rclcpp::executors::MultiThreadedExecutor`，确保 client response、power status subscription 和服务回调可以并发执行。

使用原子变量或 mutex 实现单请求互斥，并确保所有超时、异常和提前返回路径都会清除 busy 状态。

## 七、状态日志

把 power status 加入现有每秒遥测汇总，避免每帧打印：

```text
power_status=DISABLED
power_status=WAITING
power_status=STALE age=...ms
power_status=INVALID ...
power_status=OK mode=SIMULATION command_enabled=false all_enabled=false enabled=0/14 age=...ms count=...
```

当 workspace 为 STOPPED 时，power status 没有发布者属于正常情况，不要刷 ERROR；显示 `WAITING` 或 `STALE` 即可。

## 八、launch 与依赖

继续固定：

```text
ROS_DOMAIN_ID=55
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

继续清除：

```text
ROS_DISCOVERY_SERVER
CYCLONEDDS_URI
FASTRTPS_DEFAULT_PROFILES_FILE
```

更新 `jetson_planner/CMakeLists.txt` 和 `package.xml` 中所需的 `robot_control_msg` 依赖。不得删除已有 `sensor_msgs`、`std_srvs` 等依赖。

## 九、保持现有功能不变

不得破坏：

- `/planner/input` 与 `/planner/decision`
- workspace STATUS、START_SIM、START_REAL、STOP
- `enable_real_workspace_start` 真机启动门控
- telemetry 选择性订阅及动态开关
- `/planner/set_selected_joints`
- `/arm_absolute_control` client
- `enable_remote_joint_commands` 及真机关节控制门控

workspace 启动后不得自动使能电机；电机使能必须是独立、显式的请求。workspace STOP 也不要伪造电机失能回执。

## 十、编译

```bash
cd ~/joint_controller
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select robot_control_msg jetson_planner

source install/setup.bash
```

验证：

```bash
ros2 interface show robot_control_msg/msg/ArmPowerStatus
ros2 interface show robot_control_msg/srv/SetRobotPower
ros2 service type /planner/set_robot_power
ros2 topic info /arm/power_status --verbose
```

## 十一、安全测试：仅仿真

本次严禁启动 REAL 或 EtherCAT，不得将 `enable_remote_real_power_commands` 设为 true。

先验证默认拒绝：

```bash
ros2 service call /planner/set_robot_power \
  robot_control_msg/srv/SetRobotPower "{enable: true}"
```

预期 `success=false`，提示远程 power 命令未启用。

重新启动 Jetson 节点，只打开仿真所需总开关：

```bash
ros2 launch jetson_planner planner_node_jetson.launch.py \
  enable_remote_power_commands:=true
```

启动 Ubuntu 仿真：

```bash
ros2 service call /workspace/request_start_sim std_srvs/srv/Trigger "{}"
```

确认 `/workspace/status` 为 `RUNNING + SIMULATION`，且初始 `/arm/power_status` 为 14/14 状态码 64。然后测试：

```bash
ros2 service call /planner/set_robot_power \
  robot_control_msg/srv/SetRobotPower "{enable: true}"

ros2 service call /planner/set_robot_power \
  robot_control_msg/srv/SetRobotPower "{enable: false}"
```

预期使能最终为 14/14 状态码 39，失能最终为 0/14 enabled 且 14/14 状态码 64。最后调用 `/workspace/request_stop`。

额外验证：在参数总开关为 false 时，`enable=true` 必须拒绝，但 `enable=false` 仍必须允许转发并返回 Ubuntu 真实结果。

## 十二、最终输出

请最终报告：

- 修改文件
- 新增 subscription、client、service 和参数
- 多线程/callback group 实现方式
- 编译结果
- 默认使能拒绝结果
- 仿真使能和失能的 Ubuntu 服务回执
- Jetson 观察到的 14 轴最终状态
- 测试结束后 SIM/REAL 均未运行的确认

不要进行真机使能、关节运动或 EtherCAT 测试。
