# robot_lower_gateway 下位机部署与使用

从静态 IP、ROS 2/DDS、独立 overlay、systemd 自启动到更换上位机电脑的完整操作手册见
[`docs/deployment.md`](docs/deployment.md)。

本文随 `robot_lower_gateway` 功能包一起交付，适用于把当前双臂机器人下位机的反馈适配层迁移到其他相同外观、相同 14 轴结构但电机或底层通信接口可能不同的机器人。

## 1. 当前版本边界

当前版本默认是只读网关，已经实现：

- 持续订阅目标工作空间的 14 轴关节反馈；
- 按固定顺序输出 `ljoint1...ljoint7、rjoint1...rjoint7`；
- 读取电机电源、控制模式、运动、TCP 和 Cartesian execution 状态；
- 将厂商电机状态转换为统一的 `UNAVAILABLE / NOT_ENABLED / ENABLED / UNKNOWN`；
- 检查反馈是否完整、是否超时；
- 通过 pluginlib 加载机器人后端适配器；
- 作为单一 ROS 2 package 独立复制、编译和部署。

关节反馈严格按名称映射。原生 `velocity`/`effort` 完整时同步转发，确实为空时保持为空，
非空但长度错误或包含重复、未知、缺失及非有限数据时标记 `INVALID` 且不发布该帧。
Cartesian execution 状态是事件型反馈：仅 `PLANNING` 和 `EXECUTING` 要求持续新鲜；
锁存的 `IDLE` 和终态不会仅因时间经过而报错，并在新的 workspace `STARTING` 代际清除。

在只读默认值之外，当前版本还提供一个显式启用的命名空间 command proxy。它只调用目标
工作空间已经存在的 ROS service，不创建硬件对象、不实现第二套 CiA402、不写 EtherCAT
PDO。代理默认关闭，必须通过 launch 参数或配置明确打开。

代理可转发：

- `/set_robot_power`
- `/set_arm_control_mode`
- `/arm/joint_batch_control`
- `/arm_absolute_control`
- `/cartesian_increment_control`
- `/cartesian_absolute_control`

网关侧的入口默认位于 `/ubuntu_lower_gateway/` 下，因此不会和原 Ubuntu 控制服务产生
第二个全局 owner。

当前版本仍没有实现：

- Workspace START/STOP；
- 电机使能/失能命令；
- POSITION/EFFORT 模式切换；
- 单关节或 14 轴目标下发；
- 笛卡尔运动命令；
- EtherCAT PDO 写入。

因此当前包不能替代原工作空间中的 `robot_control`、controller、hardware interface 或
workspace supervisor。原控制服务继续由目标机器人原工作空间提供，proxy 只做同步转发并
返回原服务最终响应。

## 2. 安全原则

1. 网关不得创建第二个 EtherCAT master 或第二个 hardware 实例。
2. 网关不得直接写 PDO、controlword 或共享内存控制区。
3. 同一个 ROS 控制服务只能有一个 owner。
4. 当前只读阶段不得在网关中增加临时运动 publisher。
5. 反馈缺失、数组不完整、包含 NaN/Inf 或超时后，不得继续发布为有效状态。
6. 后续迁移写控制时，下位机仍然是安全检查和最终执行结果的权威来源。

## 3. 软件结构

复制单位就是完整的 `robot_lower_gateway` 目录：

```text
robot_lower_gateway/
├── CMakeLists.txt
├── package.xml
├── plugins.xml
├── README.md
├── include/robot_lower_gateway/
│   ├── backend.hpp
│   └── erobot_backend.hpp
├── src/
│   ├── robot_lower_gateway_node.cpp
│   └── erobot_backend.cpp
├── config/
│   └── erobot_v1.yaml
├── launch/
│   └── lower_gateway.launch.py
├── scripts/
│   ├── run_gateway.sh
│   └── verify_gateway_proxy.sh
└── systemd/
    └── robot-lower-gateway.service.in
```

内部关系：

```text
目标机器人原工作空间
  joint_states / power / mode / motion / TCP / execution
                         |
                         v
              ErobotBackend 或新电机 Backend
                  名称和状态归一化
                         |
                         v
                /ubuntu_lower_gateway
                  joint_states + diagnostics
```

可选控制路径（`enable_command_proxy=true`）：

```text
Jetson/测试客户端
        |
        v
/ubuntu_lower_gateway/<command>
        |
        v
目标工作空间原生 service（唯一 owner）
        |
        v
controller / hardware / EtherCAT
```

ROS package 名为 `robot_lower_gateway`，Ubuntu 运行节点名为 `/ubuntu_lower_gateway`。Jetson 已经使用 `/robot_lower_gateway`，两者不能重名。

## 4. 运行依赖

推荐环境：

```text
Ubuntu 22.04
ROS 2 Humble
RMW: rmw_fastrtps_cpp
ROS_DOMAIN_ID: 55
```

ROS 依赖由 `package.xml` 声明：

- `rclcpp`
- `pluginlib`
- `sensor_msgs`
- `geometry_msgs`
- `diagnostic_msgs`
- `robot_control_msg`
- `launch`、`launch_ros`、`ament_index_python`

`robot_control_msg` 必须与 Jetson 使用完全相同的接口版本。不能在本包中重新定义同名但字段不同的消息。目标工作空间没有该包时，应先安装相同版本的二进制接口包，或把接口仓库作为独立依赖放入工作空间。

检查接口是否可见：

```bash
source /opt/ros/humble/setup.bash
source /目标机器人工作空间/install/setup.bash

ros2 interface show robot_control_msg/msg/ArmPowerStatus
ros2 interface show robot_control_msg/msg/ArmControlModeStatus
ros2 interface show robot_control_msg/msg/CartesianExecutionStatus
```

## 5. 部署前接口调查

不要先修改 YAML。先在目标机器人原工作空间运行时记录实际接口：

```bash
ros2 node list
ros2 topic list -t
ros2 service list -t

ros2 topic info /arm/joint_states --verbose
ros2 topic info /arm/power_status --verbose
ros2 topic info /arm/control_mode_status --verbose
ros2 topic info /arm/arm_controller/motion_status --verbose
ros2 topic info /arm_tcp_pose --verbose
ros2 topic info /arm_cartesian_path_execution_status --verbose
```

至少记录：

- 14 个原生关节名称和实际顺序；
- 每个反馈 topic 的消息类型和 QoS；
- 电机未连接、失能、使能、故障时的原始状态码；
- 控制模式枚举；
- TCP 坐标系和单位；
- 各反馈正常发布频率；
- 哪个原进程是硬件和命令的唯一 owner。

如果目标接口消息类型与当前 erobot 完全不同，仅修改 YAML 不够，必须实现新 Backend，见第 14 节。

## 6. 推荐部署：独立 overlay 工作空间

这种方式不会把网关源码写入目标机器人工作空间，推荐用于正式移植。

假设：

```text
目标工作空间：/home/robot/target_robot_ws
网关工作空间：/opt/robot_lower_gateway_ws
运行用户：robot
```

创建独立工作空间：

```bash
sudo mkdir -p /opt/robot_lower_gateway_ws/src
sudo chown -R robot:robot /opt/robot_lower_gateway_ws
```

把整个功能包复制进去：

```bash
cp -a /交付目录/robot_lower_gateway \
  /opt/robot_lower_gateway_ws/src/
```

先加载目标工作空间作为 underlay，再安装依赖和编译网关：

```bash
cd /opt/robot_lower_gateway_ws
source /opt/ros/humble/setup.bash
source /home/robot/target_robot_ws/install/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install \
  --packages-select robot_lower_gateway \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

验证单包发现结果：

```bash
colcon list | grep '^robot_lower_gateway'
ros2 pkg prefix robot_lower_gateway
ros2 pkg executables robot_lower_gateway
```

预期只发现一个新 package：

```text
robot_lower_gateway
```

## 7. 简单部署：直接放入目标工作空间

研发阶段也可以直接复制到目标工作空间：

```bash
cp -a /交付目录/robot_lower_gateway \
  /home/robot/target_robot_ws/src/

cd /home/robot/target_robot_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install \
  --packages-select robot_lower_gateway

source install/setup.bash
```

这种方式不会修改其他 package 的源码，但会让网关与目标工作空间共用 `build/install/log`。正式产品更推荐第 6 节的独立 overlay。

## 8. 配置机器人 Profile

当前配置模板：

```text
config/erobot_v1.yaml
```

建议不要直接覆盖模板，复制为新机器人配置：

```bash
sudo mkdir -p /etc/robot-lower-gateway
sudo cp \
  /opt/robot_lower_gateway_ws/src/robot_lower_gateway/config/erobot_v1.yaml \
  /etc/robot-lower-gateway/robot.yaml
sudo chmod 0644 /etc/robot-lower-gateway/robot.yaml
```

配置根节点必须与运行节点一致：

```yaml
ubuntu_lower_gateway:
  ros__parameters:
```

### 8.1 关节映射

```yaml
canonical_joint_names:
  - ljoint1
  # ...
  - rjoint7

native_joint_names:
  - target_left_axis_1
  # ...
  - target_right_axis_7
```

两个数组必须：

- 长度相同；
- 名称不重复；
- 相同下标一一对应；
- 当前系统必须正好覆盖左右臂 14 个关节。

`canonical_joint_names` 是输出给上层算法的稳定名称。`native_joint_names` 是目标机器人原工作空间实际发布的名称。

### 8.2 输入 topic

```yaml
topics:
  joint_state: /arm/joint_states
  power_status: /arm/power_status
  control_mode_status: /arm/control_mode_status
  motion_status: /arm/arm_controller/motion_status
  tcp_pose: /arm_tcp_pose
  execution_status: /arm_cartesian_path_execution_status
```

如果只是 topic 名不同，可以只改 YAML。如果消息类型不同，必须新增 Backend。

### 8.3 电机状态码

```yaml
drive_status:
  enabled_codes: [39]
  unavailable_codes: [0]
```

`39` 和 `0` 只适用于当前 erobot 驱动。新电机必须根据真实反馈填写，不能直接照抄。

安全转换规则：

- 状态码属于 `unavailable_codes`：`UNAVAILABLE`；
- `enabled=true` 且状态码属于 `enabled_codes`：`ENABLED`；
- `enabled=false`：`NOT_ENABLED`；
- `enabled=true` 但状态码不属于已确认集合：`UNKNOWN`。

未知状态永远不能当作已使能。

### 8.4 超时

```yaml
stale_timeout_ms:
  joint_state: 250
  power_status: 500
  control_mode: 500
  motion_status: 500
  tcp_pose: 500
  execution: 3000
```

超时必须大于对应 topic 正常周期，并保留合理抖动余量。不要为了隐藏丢包而无限增大超时。

### 8.5 command proxy 配置

默认保持关闭：

```yaml
enable_command_proxy: false
command_proxy_prefix: "~/"
```

启用后，`"~/"` 会解析为 `/ubuntu_lower_gateway/`。原生 service 名称和等待时间可以按
目标工作空间调整：

```yaml
native_services:
  power: /set_robot_power
  mode: /set_arm_control_mode
  joint_batch: /arm/joint_batch_control
  joint_absolute: /arm_absolute_control
  cartesian_increment: /cartesian_increment_control
  cartesian_absolute: /cartesian_absolute_control

command_timeout_ms:
  power: 30000
  mode: 5000
  joint: 30000
  cartesian: 60000
```

每个原生 service 必须继续由目标工作空间的正式 controller 提供。不要把
`command_proxy_prefix` 设置为 `/`，也不要把 proxy 入口改成与原生 service 相同的全局名称。

## 9. 前台启动

先启动目标机器人原工作空间，但保持电机失能且不发送运动命令。然后另开终端：

```bash
source /opt/ros/humble/setup.bash
source /home/robot/target_robot_ws/install/setup.bash
source /opt/robot_lower_gateway_ws/install/setup.bash

export ROS_DOMAIN_ID=55
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset ROS_DISCOVERY_SERVER
unset CYCLONEDDS_URI
unset FASTRTPS_DEFAULT_PROFILES_FILE

ros2 launch robot_lower_gateway lower_gateway.launch.py \
  config_file:=/etc/robot-lower-gateway/robot.yaml
```

启动日志应包含：

```text
Lower gateway started
backend=erobot
joints=14
command_proxy=disabled
```

原控制栈没有运行时出现 `joint_state waiting` 等诊断是预期行为，不代表插件加载失败。

## 10. 输出接口

### 10.1 规范化关节状态

```text
Topic: /ubuntu_lower_gateway/joint_states
Type: sensor_msgs/msg/JointState
```

特点：

- 固定使用 `canonical_joint_names` 顺序；
- 只在一次输入消息包含全部关节且数值有效时更新；
- 输入超过 `joint_state` 超时后停止发布；
- 当前默认发布频率为 `20 Hz`。

查看一次：

```bash
ros2 topic echo /ubuntu_lower_gateway/joint_states \
  sensor_msgs/msg/JointState --once
```

检查频率：

```bash
ros2 topic hz /ubuntu_lower_gateway/joint_states
```

### 10.2 健康诊断

```text
Topic: /ubuntu_lower_gateway/diagnostics
Type: diagnostic_msgs/msg/DiagnosticArray
```

查看：

```bash
ros2 topic echo /ubuntu_lower_gateway/diagnostics \
  diagnostic_msgs/msg/DiagnosticArray --once
```

全部输入健康时应看到：

```text
level: 0
message: all configured feedback is fresh
read_only: true
command_proxy_enabled: false
joint_state_complete: true
```

诊断还包含：

- 六类反馈 age；
- 14 轴最新角度；
- 每个驱动的规范化状态和原始状态码；
- `power_command_enabled` 和 `all_drives_enabled`；
- 控制模式、运动和 execution 状态。

`all_drives_enabled=true` 只表示读取到的原控制器反馈确认全部使能，网关本身没有发送使能命令。

## 11. 默认只读安全验收

### 11.1 检查节点唯一性

```bash
ros2 node list --no-daemon --spin-time 5 | sort
```

必须只有一个：

```text
/ubuntu_lower_gateway
```

Jetson 的 `/robot_lower_gateway` 可以同时存在，这是另一个节点。

### 11.2 确认默认没有控制接口

```bash
ros2 node info /ubuntu_lower_gateway
```

允许的 publishers：

```text
/ubuntu_lower_gateway/joint_states
/ubuntu_lower_gateway/diagnostics
```

不得出现：

- `/set_robot_power` service server；
- 模式、关节或笛卡尔 service server；
- 电机、joint command、Cartesian command publisher；
- EtherCAT、controlword 或 PDO 写接口。

这是默认配置的强制安全门。只有完成下面的仿真代理验收后，才允许打开 command proxy。

### 11.3 检查 14 轴映射

```bash
ros2 topic echo /ubuntu_lower_gateway/joint_states --once
```

逐项对比原生 `/arm/joint_states`：

- 名称映射正确；
- 单位均为 rad；
- 左右臂没有交换；
- 没有 NaN/Inf；
- 当前姿态数值一致。

### 11.4 检查 stale 保护

只在仿真环境执行：停止原反馈 publisher 后，diagnostics 必须在配置超时内变为 ERROR，规范化 joint_states 必须停止更新。恢复 publisher 后应自动恢复为 OK。

不要为了 stale 测试停止正在控制真机的硬件循环。

### 11.5 稳定性观察

仿真至少连续运行 30 分钟：

```bash
ros2 topic hz /ubuntu_lower_gateway/joint_states
ros2 topic echo /ubuntu_lower_gateway/diagnostics
```

不得出现：

- 无原因反复 `stale`；
- 关节数量变化；
- 名称顺序变化；
- 网关进程退出或重复重启；
- 原控制工作空间受到影响。

### 11.6 仿真 command proxy 验收

先只启动目标 Ubuntu 仿真控制栈，确认原生 service 各只有一个 owner。然后在另一个终端
显式打开代理：

```bash
source /opt/ros/humble/setup.bash
source /home/robot/target_robot_ws/install/setup.bash
source /opt/robot_lower_gateway_ws/install/setup.bash

export ROS_DOMAIN_ID=55
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER
unset CYCLONEDDS_URI
unset FASTRTPS_DEFAULT_PROFILES_FILE

ros2 launch robot_lower_gateway lower_gateway.launch.py \
  config_file:=/etc/robot-lower-gateway/robot.yaml \
  enable_command_proxy:=true
```

代理启动日志必须显示 `command_proxy=enabled`。服务列表必须同时满足：

```text
/set_robot_power                         原控制栈唯一 owner
/ubuntu_lower_gateway/set_robot_power    网关 proxy
/ubuntu_lower_gateway/set_control_mode
/ubuntu_lower_gateway/joint_batch_control
/ubuntu_lower_gateway/joint_absolute_control
/ubuntu_lower_gateway/cartesian_increment_control
/ubuntu_lower_gateway/cartesian_absolute_control
```

也可以先运行包内的只读拓扑检查脚本：

```bash
ros2 run robot_lower_gateway verify_gateway_proxy.sh
```

它只检查六个 proxy 的 service 类型和 `/ubuntu_lower_gateway` 节点是否可发现，不会发送
任何 service 请求。

先执行无运动、仿真安全的电源失能请求，确认响应来自原服务：

```bash
ros2 service call /ubuntu_lower_gateway/set_robot_power \
  robot_control_msg/srv/SetRobotPower "{enable: false}"
```

随后按原工作空间已有的安全前置条件测试模式和运动代理。代理请求与原生请求的字段必须
完全相同，响应的 `success/accepted`、`active_mode`、`command_seq`、`target_joints` 和
`message` 必须原样返回。原生 service 不存在时，代理必须在配置的超时内返回
`native service unavailable`，不能伪造成功。

并发测试：同时发出两条代理请求，只有一条可以进入原控制服务，另一条必须返回
`another lower gateway command is already in progress`。这条互斥锁只位于网关代理，不会
锁住原 controller 的 update loop。

完成仿真 A/B 测试后，再把 Jetson 的 service 地址配置为上述网关命名空间；不要直接让网关
占用原有全局 service 名，也不要在未验收时打开 REAL 代理。

## 12. systemd 部署

当前网关默认只读，可以单独开机启动。它不会自动启动目标机器人控制栈；原控制栈未运行时
diagnostics 会保持 `waiting`。如果 systemd 配置启用了 command proxy，代理仍然只会转发
到已经存在的原生 service，不会自动启动控制栈。

### 12.1 安装配置

```bash
sudo mkdir -p /etc/robot-lower-gateway
sudo install -m 0644 \
  /opt/robot_lower_gateway_ws/src/robot_lower_gateway/config/erobot_v1.yaml \
  /etc/robot-lower-gateway/robot.yaml
```

创建 `/etc/robot-lower-gateway/gateway.env`：

```ini
ROBOT_UNDERLAY_SETUP=/home/robot/target_robot_ws/install/setup.bash
GATEWAY_OVERLAY_SETUP=/opt/robot_lower_gateway_ws/install/setup.bash
ROBOT_GATEWAY_CONFIG=/etc/robot-lower-gateway/robot.yaml
ROS_DOMAIN_ID=55
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTDDS_BUILTIN_TRANSPORTS=UDPv4
# 默认不设置；只有完成仿真验收后才显式设置 true
# ROBOT_GATEWAY_ENABLE_COMMAND_PROXY=true
# ROBOT_GATEWAY_COMMAND_PROXY_PREFIX=~/
```

权限：

```bash
sudo chmod 0644 /etc/robot-lower-gateway/gateway.env
```

### 12.2 生成 unit

模板包含四个占位符：

```text
@ROBOT_USER@
@ROBOT_GROUP@
@GATEWAY_WORKSPACE@
@GATEWAY_RUNNER@
```

以下示例使用用户和组 `robot`：

```bash
sed \
  -e 's|@ROBOT_USER@|robot|g' \
  -e 's|@ROBOT_GROUP@|robot|g' \
  -e 's|@GATEWAY_WORKSPACE@|/opt/robot_lower_gateway_ws|g' \
  -e 's|@GATEWAY_RUNNER@|/opt/robot_lower_gateway_ws/install/robot_lower_gateway/lib/robot_lower_gateway/run_gateway.sh|g' \
  /opt/robot_lower_gateway_ws/src/robot_lower_gateway/systemd/robot-lower-gateway.service.in \
  > /tmp/robot-lower-gateway.service

sudo install -m 0644 \
  /tmp/robot-lower-gateway.service \
  /etc/systemd/system/robot-lower-gateway.service
```

检查生成后的 unit 中不能残留 `@...@`：

```bash
grep -n '@' /etc/systemd/system/robot-lower-gateway.service
```

没有输出才可以继续。

### 12.3 启用和检查

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now robot-lower-gateway.service

systemctl is-enabled robot-lower-gateway.service
systemctl is-active robot-lower-gateway.service
sudo systemctl status robot-lower-gateway.service --no-pager
sudo journalctl -u robot-lower-gateway.service -n 100 --no-pager
```

正常停止和取消自启动：

```bash
sudo systemctl disable --now robot-lower-gateway.service
```

该操作只停止网关，不会停止原控制栈，也不会自动失能电机。启用 proxy 时，停止网关会
同时撤下其命名空间 service，但不会撤下原生 service owner。

## 13. 更新和回滚

当前只读版本更新前无需操作电机命令，但建议仍在仿真或机械臂静止状态进行。

```bash
sudo systemctl stop robot-lower-gateway.service

cd /opt/robot_lower_gateway_ws
source /opt/ros/humble/setup.bash
source /home/robot/target_robot_ws/install/setup.bash

colcon build --symlink-install \
  --packages-select robot_lower_gateway \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

sudo systemctl start robot-lower-gateway.service
sudo journalctl -u robot-lower-gateway.service -n 100 --no-pager
```

回滚时恢复经过验收的整个 `robot_lower_gateway` 目录和对应 YAML，再重新编译。不要只恢复 `.so` 而保留不匹配的配置或 plugin XML。

## 14. 适配不同电机和通信接口

### 14.1 只改变名称、topic 或状态码

不修改 C++，只新增 YAML profile：

- 修改 `native_joint_names`；
- 修改 `topics.*`；
- 修改 `enabled_codes` 和 `unavailable_codes`；
- 根据反馈频率修改超时。

前提是消息类型仍与当前 `robot_control_msg` 和 `sensor_msgs/JointState` 一致。

### 14.2 消息类型或通信 API 不同

在同一个功能包内新增：

```text
include/robot_lower_gateway/new_motor_backend.hpp
src/new_motor_backend.cpp
```

实现 `robot_lower_gateway::Backend`：

```cpp
std::string name() const;
BackendCapabilities capabilities() const;
void configure(rclcpp::Node &, const BackendConfiguration &);
BackendSnapshot snapshot() const;
```

然后：

1. 在 `CMakeLists.txt` 增加新共享库和厂商消息/SDK 依赖；
2. 在 `plugins.xml` 注册新类；
3. 新建 YAML profile；
4. 将 `backend_plugin` 改为新插件名；
5. 在仿真或 mock 数据上完成第 11 节全部验收。

适配器只负责读取正式接口并归一化状态，不能绕过原 controller 直接打开 EtherCAT。

### 14.3 只有裸 EtherCAT PDO

这种情况不能通过完全旁路的只读 ROS 适配器完成安全写控制。必须在目标 hardware interface 内增加正式状态/命令接口，并保证 EtherCAT master 唯一。不要让 gateway 与原驱动同时占用网卡或从站。

## 15. 常见问题

### 找不到 `robot_control_msg`

```text
Could not find a package configuration file provided by robot_control_msg
```

构建前没有 source 目标工作空间，或目标工作空间没有安装相同接口包。先解决接口依赖，不要修改 gateway 消息类型规避错误。

### 找不到 Backend 插件

```text
According to the loaded plugin descriptions the class ... does not exist
```

检查：

```bash
source /opt/ros/humble/setup.bash
source /home/robot/target_robot_ws/install/setup.bash
source /opt/robot_lower_gateway_ws/install/setup.bash

ros2 pkg prefix robot_lower_gateway
grep -n . /opt/robot_lower_gateway_ws/src/robot_lower_gateway/plugins.xml
```

重新编译并重新 source overlay，正在运行的旧进程不会自动加载新插件。

### diagnostics 一直 WAITING

- 原控制栈没有启动；
- YAML topic 名错误；
- ROS Domain/RMW/网络环境不一致；
- QoS 与原 publisher 不兼容；
- 原消息类型不同，需要新 Backend。

### joint_state incomplete

- `native_joint_names` 拼写错误；
- 原消息没有一次包含完整 14 轴；
- name 与 position 数组长度不一致；
- 某个关节值为 NaN/Inf。

### 电机状态一直 UNKNOWN

- 新电机的 enabled 状态码没有加入 profile；
- `enabled[]` 与原始状态码互相矛盾；
- 当前 adapter 不适用于该厂商消息语义。

UNKNOWN 不得通过扩大状态码集合直接伪装成 ENABLED，应先记录真实驱动状态迁移。

### 与 Jetson 节点混淆

```text
/robot_lower_gateway   Jetson 控制网关
/ubuntu_lower_gateway Ubuntu 只读适配器
```

排障时使用完整节点名，不要对 Ubuntu 节点查询 Jetson 的远程控制参数。

## 16. 最终验收清单

- [ ] 只复制了一个 `robot_lower_gateway` package。
- [ ] 目标 underlay 中存在匹配版本的 `robot_control_msg`。
- [ ] `colcon build --packages-select robot_lower_gateway` 成功。
- [ ] `/ubuntu_lower_gateway` 只有一个实例。
- [ ] 14 个 canonical/native 关节映射逐项确认。
- [ ] `/ubuntu_lower_gateway/joint_states` 为完整、有限的 14 轴反馈。
- [ ] diagnostics 为 `level=0`、`all configured feedback is fresh`。
- [ ] 电机状态码映射经过失能/使能/不可用状态验证。
- [ ] stale 测试只在仿真中完成并能自动恢复。
- [ ] 默认配置下节点没有控制 service、command publisher 或 EtherCAT owner。
- [ ] 若启用 proxy，只有 `/ubuntu_lower_gateway/*` 命名空间 service，原全局 service 仍只有一个 owner。
- [ ] proxy 仿真 A/B 验证已确认请求和最终响应均经过原控制栈；超时和并发拒绝行为已验证。
- [ ] 连续运行 30 分钟无无原因 stale、退出或重复节点。
- [ ] systemd 重启后使用正确 underlay、overlay 和 YAML。
- [ ] 原机器人工作空间功能和实时控制周期不受影响。

完成以上验收后，才能在正式现场启用命名空间 command proxy。建议顺序为电源、控制模式、
关节、笛卡尔，每完成一项都必须先确认原生 service 仍是唯一 owner。网关自身仍不直接写
EtherCAT；如果未来要删除原生 service owner，必须另行设计、评审并验证新的 controller
接口。
