# 可移植下位机网关

## 当前阶段

当前已完成第一阶段：从现有控制栈旁路、持续读取反馈，并转换为固定关节顺序和统一健康状态。
同时已实现一个默认关闭的命名空间 command proxy，用于把上位机命令转给目标工作空间
已经验证过的原生 service。

默认配置仍然是只读的：

- 不创建全局 `/set_robot_power`、关节或笛卡尔控制服务。
- 不发布任何底层命令。
- 不访问 EtherCAT PDO。
- 不启动、停止或重启现有 SIM/REAL systemd 服务。
- 可以和当前 `robot_control`、`erobot_controller` 同时运行。

proxy 打开后只提供以下命名空间入口：

```text
/ubuntu_lower_gateway/set_robot_power
/ubuntu_lower_gateway/set_control_mode
/ubuntu_lower_gateway/joint_batch_control
/ubuntu_lower_gateway/joint_absolute_control
/ubuntu_lower_gateway/cartesian_increment_control
/ubuntu_lower_gateway/cartesian_absolute_control
```

这些入口只调用目标工作空间的原生 service，并把最终响应返回；不实现第二套控制器或
EtherCAT 状态机。写接口迁移时，同一全局接口只能有一个 owner，禁止新旧节点同时提供
同名服务。

## 功能包结构

所有新增代码都位于一个可复制的 ROS 2 package：

```text
robot_lower_gateway/
├── include/robot_lower_gateway/ # 后端 API 和适配器声明
├── src/                          # 网关节点和当前 erobot 适配器
├── config/erobot_v1.yaml        # 机器人配置
├── launch/                       # 启动入口
├── scripts/                      # 独立工作空间运行入口
└── systemd/                      # 可选自启动模板
```

移植时只需要复制 `src/robot_lower_gateway/`。包内仍用 pluginlib 隔离电机适配器，但不再要求复制多个辅助 package。

现阶段继续依赖现有 `robot_control_msg`。它的 ROS 接口必须与 Jetson 保持完全一致，不能单方面修改。

## 数据流

```text
目标机器人原控制栈
  /arm/joint_states
  /arm/power_status
  /arm/control_mode_status
  motion / TCP / execution
          |
          v
robot_lower_gateway/ErobotBackend
  原生关节名 -> 标准关节名
  原始驱动码 -> 统一状态
          |
          v
robot_lower_gateway
  /ubuntu_lower_gateway/joint_states
  /ubuntu_lower_gateway/diagnostics
```

启用 proxy 后，命令流为：

```text
上位机 -> /ubuntu_lower_gateway/* -> 原生 Ubuntu service -> controller/hardware
```

网关通过订阅持续接收数据，不会通过定时 service call 轮询控制器。输出定时器只发布内存中的最新快照；关节反馈不完整或超过超时后，将停止发布规范化关节状态并在 diagnostics 中报错。

## 构建

```bash
cd /home/user/joint_controller
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --symlink-install --packages-select robot_lower_gateway

source install/setup.bash
```

## 当前机器人启动和验证

当前机器人配置位于：

```text
src/robot_lower_gateway/config/erobot_v1.yaml
```

现有 SIM 或 REAL 控制栈已经启动后，另开终端执行：

```bash
cd /home/user/joint_controller
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_DISCOVERY_SERVER
export CYCLONEDDS_URI=file:///home/user/joint_controller/cyclonedds.xml
unset FASTRTPS_DEFAULT_PROFILES_FILE

ros2 launch robot_lower_gateway lower_gateway.launch.py
```

查看固定顺序的 14 轴反馈：

```bash
ros2 topic echo /ubuntu_lower_gateway/joint_states \
  sensor_msgs/msg/JointState --once
```

查看所有输入是否完整、新鲜，以及每个电机的规范化状态：

```bash
ros2 topic echo /ubuntu_lower_gateway/diagnostics \
  diagnostic_msgs/msg/DiagnosticArray --once
```

确认没有接管控制接口：

```bash
ros2 node info /ubuntu_lower_gateway
```

其 publishers 中不得出现电机、关节或笛卡尔命令话题，service servers 中不得出现 `/set_robot_power` 等控制服务。

只读默认验证通过后，仿真中显式启动代理：

```bash
ros2 launch robot_lower_gateway lower_gateway.launch.py \
  enable_command_proxy:=true
```

然后确认 `/ubuntu_lower_gateway/*` service 存在，而原 `/set_robot_power` 等全局 service
仍各只有一个 owner。先调用代理的 `SetRobotPower(enable=false)`，再按原工作空间的安全
前置条件测试模式、关节和笛卡尔请求。响应必须与直接调用原生 service 一致；原生 service
不可用或超时时必须返回失败，不能伪造成功。

## 新机器人配置

如果新机器人仍是相同的双臂 14 轴结构，并且只改变关节名称和 topic 名称，复制 `erobot_v1.yaml` 后修改：

- `native_joint_names`
- `topics.*`
- `drive_status.enabled_codes`
- `drive_status.unavailable_codes`
- `stale_timeout_ms.*`

`canonical_joint_names` 是 Jetson 和轨迹算法看到的稳定顺序。`native_joint_names` 是目标控制栈的真实名称，两个数组必须长度相同，位置一一对应。

如果消息类型或通信方式发生变化，默认在同一个功能包内新增适配器实现，例如：

```text
include/robot_lower_gateway/new_motor_backend.hpp
src/new_motor_backend.cpp
```

新适配器实现 `robot_lower_gateway::Backend` 并通过 pluginlib 导出。只有厂商 SDK 依赖必须彻底隔离时，才把适配器单独做成外部 plugin package；默认移植交付仍保持一个 `robot_lower_gateway` package。不要在网关节点中增加按机器人型号分支。

## 接入类型判断

1. 目标控制栈已有 ROS topic/service/action：适配器只做 ROS 接口转换，不修改驱动。
2. 目标控制栈使用 `ros2_control` command/state interface：新增 controller 或使用正式 controller API，不能创建第二个 hardware 实例。
3. 目标控制栈提供共享内存或厂商 C API：适配器只连接现有控制进程，不能重复打开总线。
4. 只有裸 EtherCAT PDO：必须在目标硬件驱动内接入，无法通过完全旁路的功能包安全完成写控制。

无论哪一种情况，EtherCAT master、硬件 update loop 和 PDO 写入都必须保持唯一 owner。

## 独立工作空间部署

这个功能包可以放入单独工作空间，而不复制进目标机器人源码：

```text
/opt/robot_lower_gateway_ws/
  src/
  build/
  install/
  log/
```

网关进程按以下顺序加载环境：

```bash
source /opt/ros/humble/setup.bash
source /target/robot_ws/install/setup.bash
source /opt/robot_lower_gateway_ws/install/setup.bash
```

这只改变网关进程环境，不会修改目标工作空间的 `src`、`build` 或 `install`。

`robot_lower_gateway/scripts/run_gateway.sh` 支持以下环境变量：

| 变量 | 含义 |
|---|---|
| `ROBOT_UNDERLAY_SETUP` | 目标机器人工作空间的 `setup.bash` |
| `GATEWAY_OVERLAY_SETUP` | 网关工作空间的 `setup.bash` |
| `ROS_DOMAIN_ID` | 默认 `2` |

网关配置固定读取工作区 install/share；不再支持系统级配置副本或外部路径覆盖。

`systemd/robot-lower-gateway.service.in` 是部署模板。安装时必须将 `@GATEWAY_RUNNER@` 替换为已安装 `run_gateway.sh` 的绝对路径。模板默认不安装、不 enable，避免改变现有开机行为。

## 下一阶段门槛

只有以下条件全部满足后，才在正式现场启用 proxy：

- 当前 SIM 连续运行至少 30 分钟，diagnostics 没有无原因 stale。
- 14 轴名称和顺序确认正确。
- 电源状态在使能、失能、无硬件三种情况下转换正确。
- POSITION 模式、motion、TCP、execution 状态新鲜度正确。
- 新网关没有控制 publisher、控制 service 或 EtherCAT owner。

另外必须完成仿真 proxy A/B 测试：字段透传、最终响应透传、原生 service 不可用时的超时、
以及并发命令互斥。

启用顺序仍为电源、控制模式、关节、笛卡尔。每一项都要先确认 Jetson 指向
`/ubuntu_lower_gateway/*`，并保留原生 Ubuntu service 作为唯一 owner。
