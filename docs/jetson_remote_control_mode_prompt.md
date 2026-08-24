# Jetson 端远程机械臂控制模式修改提示词

请修改 Jetson 工作空间 `~/joint_controller`，在保留现有 workspace、遥测、关节控制和电机电源控制功能的基础上，增加机械臂 POSITION/EFFORT 控制模式的远程读取与闭环切换。

Ubuntu 端已经完成并通过仿真验证：

- 状态话题：`/arm/control_mode_status`
- 状态类型：`robot_control_msg/msg/ArmControlModeStatus`
- 高层闭环服务：`/set_arm_control_mode`
- 服务类型：`robot_control_msg/srv/SetArmControlMode`
- 控制器内部服务：`/arm/set_control_mode`，Jetson 不得直接调用
- 底层话题：`/arm_hardware_mode`，Jetson 不得直接发布
- Ubuntu SIM 启动允许 EFFORT，但仍要求 14 个电机全部失能
- Ubuntu REAL 启动禁止 EFFORT，不能通过运行时 ROS 参数修改绕过

## 一、同步接口

新增或确认 `robot_control_msg/msg/ArmControlModeStatus.msg` 内容完全一致：

```text
uint8 POSITION=0
uint8 EFFORT=1

builtin_interfaces/Time stamp

uint8 requested_mode
uint8 active_mode
bool position_command_ready

string message
```

确认 `robot_control_msg/srv/SetArmControlMode.srv` 完全一致：

```text
uint8 POSITION=0
uint8 EFFORT=1
uint8 mode
---
bool success
uint8 active_mode
string message
```

把新消息加入 `robot_control_msg/CMakeLists.txt` 的 `rosidl_generate_interfaces()`，保留所有已有接口。

## 二、Jetson 节点端点

在 `planner_node_jetson.cpp` 增加：

- `/arm/control_mode_status` subscription
- Ubuntu `/set_arm_control_mode` client
- Jetson `/planner/set_arm_control_mode` service

Jetson 对外服务和 Ubuntu client 都使用：

```text
robot_control_msg/srv/SetArmControlMode
```

状态订阅 QoS：

```cpp
rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local()
```

保存最新状态、接收时间和计数，用 mutex 保护。

Jetson 只能调用 Ubuntu `/set_arm_control_mode`，禁止：

- 发布 `/arm_hardware_mode`
- 直接调用 `/arm/set_control_mode`
- 根据请求自行伪造 active mode

## 三、参数

新增 launch/node 参数：

```text
subscribe_control_mode_status=true
control_mode_status_stale_timeout_ms=1000
control_mode_service_timeout_ms=7000
enable_remote_control_mode_commands=false
enable_remote_effort_mode_commands=false
enable_remote_real_effort_mode_commands=false
```

三个 enable 参数默认必须全部为 `false`。

## 四、状态校验与日志

收到状态时校验：

- `requested_mode` 只能是 POSITION(0) 或 EFFORT(1)
- `active_mode` 只能是 POSITION(0) 或 EFFORT(1)
- EFFORT 时 `position_command_ready` 必须为 false

把控制模式加入现有每秒遥测汇总：

```text
control_mode=DISABLED
control_mode=WAITING
control_mode=STALE age=...ms
control_mode=INVALID ...
control_mode=OK requested=POSITION active=POSITION position_ready=true age=...ms count=...
control_mode=OK requested=EFFORT active=EFFORT position_ready=false age=...ms count=...
```

workspace STOPPED 时没有发布者属于正常情况，不要每秒打印 ERROR。

## 五、POSITION 请求

POSITION 是安全回退模式：

- 不受 `enable_remote_effort_mode_commands` 限制
- 不要求电机已使能或失能
- workspace RUNNING 且 Ubuntu 服务可用时，应允许调用
- Ubuntu 返回成功后，Jetson 必须观察到新的状态满足：
  - `requested_mode=POSITION`
  - `active_mode=POSITION`

`position_command_ready=false` 不代表切换失败。它表示控制器正在保持当前位置，并等待一条新的、带时间戳的位置目标；此时不得重发旧关节命令。

## 六、EFFORT 请求门控

收到 EFFORT 请求时，任一检查失败都不得调用 Ubuntu 服务：

1. `enable_remote_control_mode_commands=true`
2. `enable_remote_effort_mode_commands=true`
3. 最新 workspace 状态为 `accepted=true + RUNNING`
4. workspace mode 必须为 SIMULATION 或 REAL，不能是 UNKNOWN
5. `/arm/control_mode_status` 结构合法且未过期
6. `/arm/power_status` 结构合法且未过期
7. 14 个电机必须全部满足：
   - `status_codes[i] == 64`
   - `enabled[i] == false`
   - `command_enabled == false`
   - `all_enabled == false`
8. 同一时刻没有其他 mode/power/joint motion 命令正在执行
9. 如果 workspace mode=REAL，还必须满足 `enable_remote_real_effort_mode_commands=true`

即使 Jetson 的真机参数全部打开，Ubuntu REAL 栈当前仍会拒绝 EFFORT。这是预期的 Ubuntu 本地安全门控，不得通过 ROS 参数或底层 topic 绕过。

建议拒绝消息：

```text
remote control mode commands are disabled
remote EFFORT mode commands are disabled
workspace is not RUNNING
workspace mode is UNKNOWN
/arm/control_mode_status is unavailable, stale, or malformed
/arm/power_status is unavailable, stale, or malformed
EFFORT requires all 14 motors disabled with status 64
another robot command is already in progress
remote REAL EFFORT mode commands are disabled
Ubuntu rejected EFFORT mode for the current workspace configuration
```

## 七、闭环返回语义

`/planner/set_arm_control_mode` 的 `success=true` 必须表示最终模式已经确认，不能只表示请求已提交。

执行流程：

1. 记录命令前 control mode status generation。
2. 调用 Ubuntu `/set_arm_control_mode`。
3. 等待 Ubuntu 最终响应。
4. Ubuntu `success=false` 时原样返回失败原因。
5. Ubuntu `success=true` 后，再等待 Jetson 收到命令后的新状态。
6. 新状态的 `requested_mode`、`active_mode` 与目标一致后才返回成功。
7. 任一阶段超时都返回 `success=false`。

成功消息示例：

```text
SIMULATION control mode confirmed as EFFORT; motors remain disabled 0/14 enabled
SIMULATION control mode confirmed as POSITION; waiting for a fresh position target
```

如果请求前状态已经新鲜且目标已满足，可以幂等返回成功。

## 八、并发与执行器

服务回调会等待 Ubuntu client 和状态 subscription，因此：

- 使用独立 callback group
- 主函数使用至少 3 线程 `MultiThreadedExecutor`
- 使用 mutex/atomic 防止并发 mode 请求
- mode 切换期间拒绝新的关节运动和电机使能请求
- 所有超时与异常路径必须释放 busy 状态

不得破坏现有电机 power 和 selected joints 的互斥逻辑。

## 九、状态机联动

- 切换到 EFFORT 成功后，不允许调用 `/planner/set_selected_joints` 或其他位置/笛卡尔运动服务。
- 切回 POSITION 成功后，如果 `position_command_ready=false`，等待用户发送一条新的目标命令；不得自动重放历史目标。
- workspace STOP/重启后清除 Jetson 本地 mode 命令 busy 状态，并等待新的状态样本。
- 不要在 workspace START 后自动切换 EFFORT。
- 不要自动使能电机。

## 十、launch 与 DDS

在 `planner_node_jetson.launch.py` 增加并传递本提示词中的新参数。

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

保留现有 workspace、telemetry、joint、power 参数和服务。

## 十一、编译

```bash
cd ~/joint_controller
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select robot_control_msg jetson_planner

source install/setup.bash
```

验证：

```bash
ros2 interface show robot_control_msg/msg/ArmControlModeStatus
ros2 interface show robot_control_msg/srv/SetArmControlMode
ros2 service type /planner/set_arm_control_mode
ros2 topic info /arm/control_mode_status --verbose
```

## 十二、安全测试：仅仿真

本次严禁 START_REAL、EtherCAT、真机电机使能和真机关节运动。

1. 默认参数启动 Jetson，EFFORT 请求必须被 Jetson 拒绝。
2. 启动 Ubuntu SIM，确认：
   - workspace RUNNING + SIMULATION
   - power status 为 14/14 状态 64
   - control mode 初始为 POSITION
3. 仅在 Jetson 启动时打开：

```bash
ros2 launch jetson_planner planner_node_jetson.launch.py \
  enable_remote_control_mode_commands:=true \
  enable_remote_effort_mode_commands:=true
```

不要打开 `enable_remote_real_effort_mode_commands`。

4. 调用：

```bash
ros2 service call /planner/set_arm_control_mode \
  robot_control_msg/srv/SetArmControlMode "{mode: 1}"
```

确认最终 `active_mode=EFFORT`，且 14 个电机仍全部为 64。

5. 切回：

```bash
ros2 service call /planner/set_arm_control_mode \
  robot_control_msg/srv/SetArmControlMode "{mode: 0}"
```

确认最终 `active_mode=POSITION`。`position_command_ready=false` 可以接受，之后必须使用新目标命令恢复位置控制。

6. STOP workspace，确认 SIM/REAL 都已停止。

## 十三、最终输出

请最终报告：

- 修改文件
- 新增 subscription、client、service 和参数
- 并发互斥实现
- 编译结果
- 默认 EFFORT 拒绝结果
- 仿真 EFFORT/POSITION 最终回执
- 切换期间 14 轴电源保持失能的证据
- 测试结束后 SIM/REAL 状态

不要进行任何真机测试，不要修改 Ubuntu 代码。
