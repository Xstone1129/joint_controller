# Jetson 端远程笛卡尔单点控制修改提示词

请修改 Jetson 工作空间 `~/joint_controller`，在保留现有 workspace、遥测、选择性关节控制、电机电源控制和控制模式切换功能的基础上，增加双臂笛卡尔单点绝对/增量控制。

Ubuntu 端已经实现并通过仿真闭环验证：

- 绝对控制服务：`/cartesian_absolute_control`
- 增量控制服务：`/cartesian_increment_control`
- 服务类型分别为：
  - `robot_control_msg/srv/CartesianAbsoluteControl`
  - `robot_control_msg/srv/CartesianIncrementControl`
- TCP 反馈：`/arm_tcp_pose`，类型 `robot_control_msg/msg/EndEffectorPose`
- 执行状态：`/arm_cartesian_path_execution_status`，类型 `robot_control_msg/msg/CartesianExecutionStatus`
- 运动状态：`/arm/arm_controller/motion_status`，类型 `robot_control_msg/msg/ArmMotionStatus`
- Ubuntu 服务自身会校验 power、POSITION 模式、反馈新鲜度、数值范围和 planner busy 状态，并等待 `STREAM_FINISHED`、运动停止和 TCP 到位后才返回 `success=true`

Ubuntu 仿真实测：左臂 X 增量 5 mm 最终误差约 `0.000408 m`；绝对回位最终误差约 `0.000032 m`。

## 一、同步接口

确认 Jetson 的 `robot_control_msg/srv/CartesianAbsoluteControl.srv` 与 Ubuntu 完全一致：

```text
float64 lx
float64 ly
float64 lz
float64 lroll
float64 lpitch
float64 lyaw
float64 lqx
float64 lqy
float64 lqz
float64 lqw

float64 rx
float64 ry
float64 rz
float64 rroll
float64 rpitch
float64 ryaw
float64 rqx
float64 rqy
float64 rqz
float64 rqw

float64 vel
float64 acc
---
bool success
string message
```

确认 `robot_control_msg/srv/CartesianIncrementControl.srv` 字段与上述接口完全相同，语义为位置/姿态增量。

确认以下消息与 Ubuntu 一致，并且所有接口都在 `robot_control_msg/CMakeLists.txt` 的 `rosidl_generate_interfaces()` 中：

```text
# EndEffectorPose.msg
geometry_msgs/Pose left_ee_pose
geometry_msgs/Pose right_ee_pose
```

```text
# CartesianExecutionStatus.msg
builtin_interfaces/Time stamp

uint8 IDLE=0
uint8 PLANNING=1
uint8 EXECUTING=2
uint8 STREAM_FINISHED=3
uint8 FAILED=4
uint8 REJECTED_BUSY=5

uint8 state
uint32 planned_points
float64 planned_duration_sec
string message
```

不要改变服务字段或另造一套不兼容的 Jetson 专用笛卡尔接口。

## 二、Jetson 节点端点

在 `planner_node_jetson.cpp` 中增加两个 Ubuntu service client：

```text
/cartesian_absolute_control
  robot_control_msg/srv/CartesianAbsoluteControl

/cartesian_increment_control
  robot_control_msg/srv/CartesianIncrementControl
```

增加两个 Jetson 对外服务：

```text
/planner/set_cartesian_absolute
  robot_control_msg/srv/CartesianAbsoluteControl

/planner/set_cartesian_increment
  robot_control_msg/srv/CartesianIncrementControl
```

Jetson 只能调用 Ubuntu 的两个高层服务。严禁 Jetson：

- 直接发布 `/arm_cartrsian_position_cmd`（注意 Ubuntu 原工程中的 topic 拼写就是 `cartrsian`）
- 直接调用或绕过 Ubuntu 内部规划节点
- 向 `/arm/joint_states` 发布伪反馈
- 只凭“命令已提交”返回成功

## 三、参数和默认安全状态

新增 node/launch 参数：

```text
enable_remote_cartesian_commands=false
enable_remote_real_cartesian_commands=false
cartesian_service_timeout_ms=70000
cartesian_feedback_stale_timeout_ms=1000
cartesian_position_confirmation_tolerance_m=0.002
cartesian_orientation_confirmation_tolerance_rad=0.03
max_remote_cartesian_velocity_mps=0.50
max_remote_cartesian_acceleration_mps2=1.0
max_remote_cartesian_increment_translation_m=0.05
max_remote_cartesian_increment_rotation_rad=0.35
max_remote_cartesian_absolute_translation_delta_m=0.20
max_remote_cartesian_absolute_rotation_delta_rad=0.75
```

所有远程笛卡尔运动默认关闭。仿真测试必须显式设置：

```text
enable_remote_cartesian_commands=true
subscribe_tcp_pose=true
subscribe_execution_status=true
```

真机还必须额外显式设置 `enable_remote_real_cartesian_commands=true`。本次禁止打开该参数。

上述 Jetson 限制不能宽于 Ubuntu 当前限制。运行时参数服务不得用于放宽安全上限；可以像 Ubuntu 一样关闭参数服务，或通过参数回调拒绝对安全参数的运行时修改。

## 四、命令前门控

绝对和增量服务共用一套门控，任一检查失败都不得调用 Ubuntu：

1. `enable_remote_cartesian_commands == true`
2. 最新 workspace 状态为 `accepted=true + RUNNING`
3. workspace mode 为 `SIMULATION` 或 `REAL`，不能是 `UNKNOWN`
4. workspace mode 为 `REAL` 时，`enable_remote_real_cartesian_commands == true`
5. 最新 `/arm/power_status` 合法、新鲜且 14/14 电机均为状态码 39
6. 最新 `/arm/control_mode_status` 合法、新鲜，`active_mode=POSITION` 且 `position_command_ready=true`
7. 最新 `/arm_tcp_pose`、motion status 和 Cartesian execution status 均已收到且未过期
8. Cartesian execution state 不是 `PLANNING` 或 `EXECUTING`
9. 同一时刻没有 joint、Cartesian、power 或 mode 命令正在执行
10. 所有请求数值均为有限数，`vel > 0`、`acc > 0`，且不超过 Jetson 安全上限

增量命令额外校验：

- 每只手的平移增量范数不超过 `0.05 m`
- 每只手的旋转增量不超过 `0.35 rad`
- 四元数增量全为零时使用 RPY 增量
- 使用四元数时必须是有效、接近单位范数的四元数

绝对命令额外校验：

- 左右臂位置和姿态都必须完整提供
- 四元数必须有效且接近单位范数；若项目保留 RPY/四元数二选一语义，必须与 Ubuntu 完全一致
- 相对当前 TCP，每只手平移变化不超过 `0.20 m`
- 相对当前 TCP，每只手旋转变化不超过 `0.75 rad`

建议拒绝消息明确指出原因，例如：

```text
remote Cartesian commands are disabled
workspace is not RUNNING
workspace mode is UNKNOWN
real Cartesian commands are disabled
Cartesian motion requires all 14 motors enabled
Cartesian motion requires POSITION mode and position_command_ready=true
TCP or execution feedback is unavailable or stale
another robot command is already in progress
Cartesian increment exceeds the configured safety limit
```

## 五、共享互斥和避免死锁

两个 Jetson 笛卡尔服务必须共享同一个 busy 状态，并与现有 joint、power、mode 命令形成全局机器人命令互斥：

- 笛卡尔运动中拒绝新的关节、笛卡尔、power enable 和 mode 切换请求
- power disable 属于安全动作，可以按现有设计优先处理，但不能伪造执行结果
- workspace STOP 后清除本地 busy，并让正在等待的运动请求失败退出
- 所有超时、异常和提前返回路径都必须释放 busy，可使用 RAII guard

服务回调需要等待 Ubuntu client response 和状态 subscription，因此必须：

- 使用独立 callback group
- 使用至少 3 线程的 `rclcpp::executors::MultiThreadedExecutor`
- client response、TCP/status subscription 与外部服务不能都放在同一个 MutuallyExclusive callback group

## 六、最终响应和交叉确认

`/planner/set_cartesian_absolute` 与 `/planner/set_cartesian_increment` 的 `success=true` 必须表示动作最终完成，不能只表示 client 请求已发送。

执行流程：

1. 保存命令前的 TCP 位姿、TCP 接收 generation、execution generation 和目标模式。
2. 对增量请求，基于命令前 TCP 计算期望的绝对目标，供最终交叉确认使用。
3. 调用对应 Ubuntu 高层服务。
4. 最长等待 `cartesian_service_timeout_ms`；该值不能小于 Ubuntu 当前最长完成时间 60 秒并预留 DDS 余量。
5. Ubuntu 返回 `success=false` 时，Jetson 原样返回失败，并保留 Ubuntu message。
6. Ubuntu 返回 `success=true` 后，Jetson 仍必须确认收到命令后的新 TCP 和 execution status。
7. execution status 必须为 `STREAM_FINISHED`，motion status 必须满足 `is_moving=false + goal_reached=true`。
8. 左右 TCP 相对期望目标的位置误差都不超过 `0.002 m`，姿态误差都不超过 `0.03 rad`，才返回 `success=true`。
9. 如果 Ubuntu 成功但 Jetson 未观察到新反馈、状态不一致或超时，返回 `success=false`，明确写出交叉确认失败原因。

成功消息需要保留 Ubuntu 回执并附 Jetson 独立误差，例如：

```text
SIMULATION Cartesian increment completed and cross-confirmed; left_pos_error=...m left_rot_error=...rad right_pos_error=...m right_rot_error=...rad; Ubuntu: ...
```

不得使用旧的 transient-local `STREAM_FINISHED` 或旧 TCP 样本确认新命令，必须比较 generation/接收序号。

## 七、状态订阅与 QoS

复用现有遥测 subscription 时，保证命令功能所需端点真实存在。若用户动态关闭 TCP 或 execution telemetry，新的笛卡尔命令必须拒绝，不能在无交叉确认时继续运动。

QoS 与 Ubuntu 发布端保持兼容：

```text
/arm_tcp_pose
  RELIABLE, KEEP_LAST(1), VOLATILE

/arm_cartesian_path_execution_status
  RELIABLE, KEEP_LAST(1), TRANSIENT_LOCAL

/arm/arm_controller/motion_status
  RELIABLE, KEEP_LAST(10), VOLATILE

/arm/power_status
  RELIABLE, KEEP_LAST(1), TRANSIENT_LOCAL

/arm/control_mode_status
  RELIABLE, KEEP_LAST(1), TRANSIENT_LOCAL
```

在现有每秒遥测汇总中保留 TCP/execution 状态，并在命令完成或失败时单独打印一次结果；不要每帧打印。

## 八、统一配置预设与 robotctl

不要要求操作员每次手工 `source`、`export`、输入多行 launch 参数或直接拼接长 ROS 2 service 请求。实现一个统一的 Jetson 控制入口：

```text
robotctl
```

`robotctl` 必须是有类型的 Python/rclpy CLI，直接创建 ROS service client 和 subscription。禁止通过 `os.system("ros2 ...")` 调用 CLI 后解析人类可读输出。

### 8.1 安装和环境封装

新增：

```text
jetson_planner/config/planner_monitor.yaml
jetson_planner/config/planner_sim_remote.yaml
jetson_planner/config/planner_real_remote.yaml
jetson_planner/scripts/robotctl.py
jetson_planner/scripts/run_jetson_planner_profile.sh
jetson_planner/systemd/jetson-planner@.service
jetson_planner/scripts/install_robotctl.sh
```

安装脚本只需执行一次，完成以下工作：

- 把 `robotctl` 安装或软链接到 `~/.local/bin/robotctl`
- 安装并 reload systemd user unit
- 不使用 root 运行 planner ROS 节点
- 检查 `~/.local/bin` 是否在 PATH；不在时打印一条明确的修复命令

更新 `jetson_planner/CMakeLists.txt`，安装 Python CLI、profile runner、配置文件和 user unit。不要依赖从源码目录直接运行。更新 `package.xml`，至少声明 `rclpy`、`std_srvs`、`robot_control_msg` 以及 CLI 实际导入的消息包为运行依赖。

推荐安装布局：

```text
install/jetson_planner/lib/jetson_planner/robotctl.py
install/jetson_planner/lib/jetson_planner/run_jetson_planner_profile.sh
install/jetson_planner/share/jetson_planner/config/*.yaml
install/jetson_planner/share/jetson_planner/systemd/jetson-planner@.service
```

`install_robotctl.sh` 必须从 `ros2 pkg prefix jetson_planner` 或 ament index 解析安装前缀，不能假定当前工作目录就是源码目录。

systemd user unit 和所有 wrapper 内部必须自行设置：

```text
ROS_DOMAIN_ID=55
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

并自行 source：

```text
/opt/ros/humble/setup.bash
/home/yuling/joint_controller/install/setup.bash
```

用户正常使用 `robotctl` 时不得再要求手工 source 或 export。

### 8.2 启动预设

三个参数预设语义如下：

```text
monitor
  所有远程写命令关闭
  telemetry 订阅保持启用

sim
  enable_remote_joint_commands=true
  enable_remote_power_commands=true
  enable_remote_control_mode_commands=true
  enable_remote_cartesian_commands=true
  subscribe_tcp_pose=true
  subscribe_execution_status=true
  所有 enable_remote_real_* 参数=false
  enable_real_workspace_start=false

real
  单独文件，允许显式真机操作
  不得被默认选择
  robotctl 必须要求 --confirm-real 才能启动
```

本次只启动和测试 `monitor`、`sim`，禁止启动 `real` profile。

支持：

```bash
robotctl up monitor
robotctl up sim
robotctl up real --confirm-real
robotctl down
robotctl logs
```

`robotctl up` 必须保证最多只有一个 `planner_node_jetson` 实例。切换 profile 前停止其他 profile，启动后等待节点和必要服务出现；失败返回非零退出码并打印具体缺失端点。

`robotctl logs` 等同于跟踪当前 user service 日志，不要求用户查找 PID 或 ROS 日志目录。

### 8.3 单动作命令

至少支持：

```bash
robotctl status
robotctl start sim
robotctl start real --confirm-real
robotctl stop

robotctl power on
robotctl power off
robotctl mode position
robotctl mode effort

robotctl joint ljoint6 0.10 --relative --vel 0.20 --acc 0.20

robotctl cart-inc left --x 0.005 --vel 0.05 --acc 0.10
robotctl cart-inc right --z -0.005 --vel 0.05 --acc 0.10

robotctl pose save home
robotctl cart-abs home --vel 0.05 --acc 0.10
```

要求：

- `status` 一次输出 workspace、mode、power、joint/TCP、motion、execution 和 planner profile 摘要
- `start` 调用现有 workspace Trigger 服务，并等待 `/workspace/status` 达到对应 `RUNNING + mode`
- `stop` 等待最终 `STOPPED`
- power/mode/joint/Cartesian 命令调用对应 `/planner/*` 高层服务并打印最终响应
- 任一失败返回非零进程退出码，便于脚本和上层程序判断
- 不在 shell 中手工拼 YAML service request
- 所有数值使用 argparse 类型校验，并拒绝 NaN/Inf
- `cart-inc left` 必须自动把右臂增量填零，反之亦然
- `pose save NAME` 保存当前左右 TCP 的完整位置和单位四元数到用户配置目录
- `cart-abs NAME` 从保存的 pose 读取双臂完整目标，避免操作员输入 20 个字段
- 保存 pose 时使用原子写入，并记录保存时间、workspace mode 和来源

`robotctl start sim` 只启动 workspace，不得自动使能电机。`robotctl power on` 仍是显式动作。

### 8.4 一键安全会话

为了减少日常仿真操作命令，再提供两个明确的复合命令：

```bash
robotctl ready sim
robotctl safe-stop
```

`robotctl ready sim` 依次执行并逐项等待最终状态：

1. 确保 planner 使用 `sim` profile 运行。
2. 请求 Ubuntu `START_SIM`。
3. 等待 `RUNNING + SIMULATION`。
4. 请求 POSITION 模式并确认 active mode。
5. 请求仿真电机使能并确认 14/14 状态码 39。
6. 确认 JointState、TCP、motion 和 execution 反馈新鲜。
7. 输出一行 `SIMULATION READY`。

任一步失败时必须执行安全回滚：尽力 power off，再 STOP simulation，并返回非零退出码。

`robotctl safe-stop` 必须：

1. 即使其他写命令开关关闭，也尽力调用 power off。
2. 等待 0/14 enabled 或明确报告无法确认。
3. 调用 workspace STOP 并等待 STOPPED。
4. 不启动 REAL，不自动切换 profile。

真机不得使用普通 `ready sim` 逻辑隐式替换。未来 `robotctl ready real --confirm-real` 必须同时要求交互确认，默认拒绝非交互执行；本次不要实现或测试自动真机 ready。

### 8.5 日常最短操作路径

完成后，仿真单点控制应只需要：

```bash
robotctl ready sim
robotctl cart-inc left --x 0.005
robotctl safe-stop
```

默认 `cart-inc` 参数使用安全低速：

```text
vel=0.05 m/s
acc=0.10 m/s^2
```

仍保留完整参数选项，不能因为封装而绕过节点和 Ubuntu 的安全校验。

## 九、launch 与 DDS

把所有新增参数加入 `planner_node_jetson.launch.py` 并传给节点。

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

不得改变已有 workspace mode、真机启动、power、control mode 和 selected joints 的安全参数默认值。

## 十、保持现有功能不变

不得破坏：

- `/planner/input` 与 `/planner/decision`
- workspace STATUS、START_SIM、START_REAL、STOP
- `enable_real_workspace_start` 门控
- telemetry 选择性订阅和动态开关
- `/planner/set_selected_joints` 与 `/arm_absolute_control` client
- `/planner/set_robot_power`
- `/planner/set_arm_control_mode`
- 所有 SIM/REAL 双重门控与最终状态确认

不要让 workspace START 自动使能电机或自动执行笛卡尔命令。

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
ros2 interface show robot_control_msg/srv/CartesianAbsoluteControl
ros2 interface show robot_control_msg/srv/CartesianIncrementControl
ros2 service type /planner/set_cartesian_absolute
ros2 service type /planner/set_cartesian_increment
ros2 topic info /arm_tcp_pose --verbose
ros2 topic info /arm_cartesian_path_execution_status --verbose
```

## 十二、安全测试：仅仿真

本次严禁调用 START_REAL、启动 EtherCAT、设置 `enable_remote_real_cartesian_commands=true` 或执行真机运动。

1. 默认参数启动 Jetson，调用两个新服务都必须被 Jetson 拒绝，原因是远程笛卡尔命令未启用。
2. 只启动 Ubuntu SIM，确认 workspace 为 `RUNNING + SIMULATION`。
3. 显式切到 POSITION 模式，并通过现有 Jetson power 服务使能仿真电机，确认 14/14 状态码为 39。
4. 安装统一入口并启动仿真 profile：

```bash
cd ~/joint_controller
./src/jetson_planner/scripts/install_robotctl.sh
robotctl up sim
```

5. 先测试明显超限的 0.20 m 增量，必须在 Jetson 本地拒绝，Ubuntu 服务不应收到请求。
6. 测试左臂 X 正向 0.005 m、右臂保持不变、`vel=0.05`、`acc=0.10`。
7. 只有 Ubuntu 返回成功且 Jetson 观察到新的 `STREAM_FINISHED`、motion stopped 和 TCP 到位，外部服务才能返回成功。
8. 使用绝对服务回到测试前保存的左右 TCP 基线，再次完成交叉确认。
9. 用 `robotctl safe-stop` 失能仿真电机并 STOP workspace，确认 SIM/REAL stack 均 inactive。
10. 另外完整验证一次最短操作路径：

```bash
robotctl ready sim
robotctl cart-inc left --x 0.005
robotctl safe-stop
```

三条命令都必须根据最终状态返回正确的进程退出码。

调用示例中的其余字段必须按接口完整填写；不要用零四元数作为绝对姿态。绝对回位应使用测试前实际读取的左右位置和单位四元数。

## 十三、最终输出

请最终报告：

- 修改文件
- 新增 clients、services、参数和 callback groups
- 新增参数预设、systemd user unit 和 `robotctl` 子命令
- 全局命令互斥实现
- Ubuntu 最终响应与 Jetson TCP/execution 交叉确认逻辑
- 编译结果
- 默认关闭时的拒绝结果
- 0.20 m 超限拒绝结果
- 5 mm 仿真增量最终回执与误差
- 仿真绝对回位最终回执与误差
- `robotctl ready sim`、`cart-inc`、`safe-stop` 三条命令的实际输出
- 测试结束后 supervisor、SIM、REAL 服务状态
