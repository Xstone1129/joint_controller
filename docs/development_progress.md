# 双臂机器人控制项目开发进度文档

> 文档用途：记录当前工作空间的开发范围、实现状态、验证证据、遗留问题和后续计划，作为阶段汇报、联调记录和现场验收的基础文档。
>
> 统计基准：2026-08-07。本文根据当前工作空间源码、配置、启动文件、测试代码、测试结果文件和现有项目说明整理。
>
> 重要说明：本文不把“代码已实现”直接等同于“整机已验收”。所有需要真实机器人、真实负载、真实通信链路或安全设备参与的结果均保留为空，由现场测试后填写。

## 1. 文档信息

| 项目 | 内容 |
|---|---|
| 项目名称 | Joint Controller 2.0 / 双臂机器人 ROS 2 控制系统 |
| 工作空间 | `/home/user/joint_controller` |
| 目标机器人 | 双臂 14 轴机器人，另包含腰部和升降等外围机构接口 |
| 主要软件平台 | Ubuntu 22.04、ROS 2 Humble |
| 默认 ROS 域 | `ROS_DOMAIN_ID=55` |
| 默认 RMW | `rmw_fastrtps_cpp` |
| 文档版本 | v0.1 |
| 文档编写人 |  |
| 项目负责人 |  |
| 当前阶段 | 控制软件和下位机部署体系已形成，仿真及模块级验证已有基础，整机最终结果待现场填写 |
| 下一次更新日期 |  |

### 1.1 状态标记说明

本文使用以下状态：

- `已实现`：源码、配置或启动入口已经存在，能够从代码中确认功能边界。
- `已有记录`：现有项目文档或构建测试结果中有验证记录，但本次未重新完成全部现场流程。
- `部分完成`：核心路径已经具备，但仍有接口、异常路径、部署或实机条件未闭环。
- `待验证`：需要重新执行测试或补充客观数据。
- `待填写`：刻意留空，专门用于填写整机或现场结果。
- `存在问题`：当前已有明确失败、风险或工程缺口。

## 2. 项目目标与范围

### 2.1 总体目标

构建一套面向双臂 14 轴机器人的 ROS 2 控制系统，提供以下能力：

1. 机器人 URDF、网格模型、关节和末端坐标系描述。
2. 基于 `ros2_control` 的仿真控制和真实硬件控制。
3. 双臂关节空间绝对控制、批量关节控制。
4. 双臂笛卡尔空间绝对控制、增量控制和多路点路径控制。
5. 基于 Pinocchio 的运动学计算和基于 Ruckig 的轨迹规划。
6. 电机电源、控制模式、运动状态、TCP 位姿和路径执行状态的闭环反馈。
7. Workspace 的 SIMULATION / REAL 启动、停止和状态管理。
8. Jetson 上位机与 Ubuntu 下位机之间的远程控制和安全门控。
9. 可移植的下位机反馈网关，以及默认关闭的命名空间 command proxy。
10. EtherCAT 升降机构、USB-FDCAN 腰部等外围硬件的独立控制接口。
11. systemd 自启动、仿真/真实栈互斥、运行脚本和标准化验收流程。

### 2.2 当前明确不应混淆的边界

- `robot_control` 负责高层控制服务、运动规划输入和运动完成确认。
- `erobot_controller` 负责机械臂 ros2_control 控制器和轨迹执行。
- `erobot_hw` 是当前机械臂硬件接口，包含仿真/真实模式和共享内存通信路径。
- `joint_hardware` 面向升降和腰部等外围关节，不等同于机械臂硬件接口。
- `robot_lower_gateway` 默认只读，不能替代原有控制器、硬件接口或 EtherCAT master。
- Jetson 上的 `/robot_lower_gateway` 与 Ubuntu 上的 `/ubuntu_lower_gateway` 是两个不同节点。
- 失能、急停和物理安全链路不能由 ROS 服务响应单独代替。

## 3. 当前进度总览

| 子系统/能力 | 当前状态 | 当前依据 | 仍需补充 |
|---|---|---|---|
| ROS 2 工作空间和包结构 | 已实现 | 8 个 ROS 包、第三方规划代码、脚本和 systemd 文件已存在 | 清理包元数据和目录命名，补齐统一版本信息 |
| 上下位机中间件通信 | 已实现/持续优化 | Jetson 与 Ubuntu 双机 ROS 2、DDS、静态 IP 和 systemd 部署体系已建立 | 持续优化网络稳定性、反馈新鲜度、启动互斥和远程控制链路 |
| 机器人模型与 URDF | 已实现 | `robot_arm_description`、`erobot_controller/config/robot_arm` 含 URDF/Xacro、网格和 RViz 配置 | 实机零位、工具坐标系、负载参数现场确认 |
| 双臂仿真控制栈 | 已实现/已有记录 | `joint_controller_stack_sim.launch.py`、仿真硬件路径和 RViz 启动入口 | 重新执行完整启动、运动、停止回归 |
| 双臂真实控制栈 | 已实现 | systemd real unit、真实硬件接口和部署文档已存在 | 现场 EtherCAT、急停、零位、运动和负载验收 |
| 关节绝对控制 | 已实现/已有记录 | `JointAbsoluteControl` 服务和对应高层节点 | 补充 14 轴逐轴、边界和异常测试数据 |
| 批量/选择性关节控制 | 已实现 | `JointBatchControl`、批量执行状态和 workflow 支持 | 补充并发、乱序、重复关节和超时数据 |
| 笛卡尔单点控制 | 已实现/已有记录 | 绝对/增量服务、TCP 反馈、运动完成确认 | 补充双臂同时运动、姿态、奇异位形和真机数据 |
| 笛卡尔多路点路径 | 已实现 | 绝对/增量路径服务、waypoint 和 blend radius 接口 | 补充连续路径误差、拐角、失败恢复和真机结果 |
| IK 与轨迹规划 | 已实现 | Pinocchio、Ruckig、统一 MoveL 节点和规划失败诊断 | 补充边界位姿、奇异点、限位和压力测试 |
| 电源闭环 | 已实现/已有记录 | `/set_robot_power`、状态码 39/64、最终反馈确认 | 整机失能时延、掉线、故障电机和物理急停数据 |
| 控制模式闭环 | 已实现/已有记录 | POSITION/EFFORT、状态反馈和请求门控 | 真机模式切换和失效恢复数据 |
| Workspace supervisor | 已实现/已有记录 | SIM/REAL systemd 管理、状态机和 source 白名单 | 重启、异常退出、并发请求和断电恢复验收 |
| 升降 EtherCAT | 已实现/模块测试通过 | CiA 402、PDO、单位转换、轨迹、零偏和 mock 集成测试 | 真机从站、制动器、限位和负载测试 |
| 腰部 USB-FDCAN | 已实现/模块测试通过 | 初始化、反馈、写入、清错、置零和桥接测试 | 真机 CAN 总线、零位和整机联动测试 |
| 下位机反馈网关 | 第一阶段已实现 | 反馈适配、14 轴映射、diagnostics、pluginlib | 现场 DDS、连续运行和目标机器人迁移验收 |
| 下位机 command proxy | 已实现/待仿真 A/B | 默认关闭的命名空间 service 转发 | 仿真 A/B、超时、并发、失败透传和正式启用门槛 |
| 自动化 workflow | 已实现 | YAML 姿态、关节分组、笛卡尔增量、确认点、安全停止 | 扩充真实任务脚本和任务级统计 |
| 抓箱子任务及外设测试 | 持续优化 | 已完成腰部、升降模块测试及负压测试，测试后电机温度状态良好 | 补充整机抓取成功率、负载、循环次数和温度实测值 |
| 底盘结构与整机安装 | 已完成阶段性改良 | 已完成底盘重心、承重、理线和拖链安装改良 | 联动运行时继续确认线束干涉、拖链弯曲半径和结构稳定性 |
| 整机性能和任务结果 | 待填写 | 当前工作空间没有可作为最终结论的完整整机数据表 | 由现场测试填写本文件第 9、10、11 节 |

## 4. 系统架构与数据流

### 4.1 上下位机角色

| 设备 | 当前地址/工作空间记录 | 主要职责 |
|---|---|---|
| Jetson 上位机 | `192.168.2.10/24`；`/home/yuling/robot_gateway_ws` | `robotctl`、远程命令、安全门控、状态汇总和最终结果交叉确认 |
| Ubuntu 下位机 | `192.168.2.20/24`；`/home/user/joint_controller` | ros2_control、IK、轨迹执行、硬件访问、真实执行和最终安全检查 |
| Ubuntu 反馈网关 | 当前工作空间内，节点名 `/ubuntu_lower_gateway` | 反馈归一化、健康诊断和可选命名空间 service 透传 |

### 4.2 机械臂主链路

```text
Jetson planner / robotctl
        |
        | ROS 2 service，ROS_DOMAIN_ID=55
        v
Ubuntu robot_control 高层服务
        |
        | 关节目标或 TCP 目标/路径
        v
统一 IK / MoveL / 轨迹规划节点
        |
        | Robotarmjoint / Robotarmmovel / Robotarmmovelpath
        v
erobot_controller + controller_manager
        |
        v
erobot_hw / 仿真硬件 / 真实硬件通信
        |
        v
机械臂 14 轴
```

反馈链路：

```text
机械臂硬件或仿真控制器
  -> /arm/joint_states
  -> /arm/power_status
  -> /arm/control_mode_status
  -> /arm/arm_controller/motion_status
  -> /arm_tcp_pose
  -> /arm_cartesian_path_execution_status
 -> 高层服务闭环确认 / Jetson 遥测 / Ubuntu gateway
```

### 4.5 上下位机中间件通信部署与优化

上下位机采用 Jetson 上位机与 Ubuntu 下位机分工协作的 ROS 2 通信架构，统一使用 `ROS_DOMAIN_ID=55`、`rmw_fastrtps_cpp` 和 UDPv4。Jetson 负责远程入口、安全门控和状态汇总，Ubuntu 负责控制器、规划、硬件访问和最终执行确认；Ubuntu 反馈网关默认只读，避免产生第二个硬件或控制 owner。

当前已完成静态 IP、DDS 环境、双端部署文档、systemd 启动互斥和反馈 stale 保护的基础建设。后续重点优化：

- 统一双端环境加载和启动顺序，减少 DDS 配置不一致。
- 检查 topic QoS、反馈新鲜度、网络断开和服务超时行为。
- 保证 SIM/REAL、原生控制服务和 gateway proxy 不产生重复 owner。
- 继续完善远程命令的最终状态确认和异常恢复。

### 4.3 Workspace 生命周期

Workspace 控制接口使用 `WorkspaceControl` 和 `WorkspaceStatus`：

```text
STOPPED -> STARTING -> RUNNING -> STOPPING -> STOPPED
                         |
                         v
                        ERROR
```

启动时必须明确选择 `SIMULATION` 或 `REAL`。默认仿真入口不会自动打开电机电源；真实机器人必须在硬件、急停、反馈和失能状态确认后，再按“启动 workspace → 确认模式 → 确认反馈 → 电源操作”的顺序执行。

### 4.4 控制 owner 原则

当前设计要求：

- EtherCAT master、PDO update loop 和硬件实例只能有一个 owner。
- 同名全局控制服务只能有一个 owner。
- gateway 不能直接写 EtherCAT PDO、controlword 或共享内存控制区。
- gateway proxy 只调用目标工作空间已有的原生 service。
- Jetson 不能伪造 `/arm/joint_states`、电源状态、模式状态或执行完成状态。

## 5. ROS 包和代码模块进度

### 5.1 `robot_control_msg`：消息和服务接口

位置：[`src/robot_control_msg`](/home/user/joint_controller/src/robot_control_msg)

已形成双臂控制所需的核心接口：

| 接口 | 类型 | 用途 | 状态 |
|---|---|---|---|
| `JointAbsoluteControl` | service | 双臂 14 轴绝对关节位置 | 已实现 |
| `JointBatchControl` | service | 选择关节、批量执行、相对/绝对控制 | 已实现 |
| `SelectedJointControl` | service | 选择性关节控制兼容接口 | 已实现 |
| `CartesianAbsoluteControl` | service | 双臂 TCP 绝对位姿 | 已实现 |
| `CartesianIncrementControl` | service | 双臂 TCP 增量位姿 | 已实现 |
| `CartesianPathAbsoluteControl` | service | 双臂多路点绝对路径 | 已实现 |
| `CartesianPathIncrementControl` | service | 双臂多路点增量路径 | 已实现 |
| `SetRobotPower` | service | 电机使能/失能 | 已实现 |
| `SetArmControlMode` | service | POSITION/EFFORT 模式切换 | 已实现 |
| `ArmPowerStatus` | message | 电机状态码、使能数量和电源命令状态 | 已实现 |
| `ArmControlModeStatus` | message | 请求模式、实际模式、位置命令就绪状态 | 已实现 |
| `ArmMotionStatus` | message | 是否运动、目标是否到达 | 已实现 |
| `CartesianExecutionStatus` | message | 规划、执行、流结束、失败和 busy 状态 | 已实现 |
| `EndEffectorPose` | message | 左右 TCP 位姿反馈 | 已实现 |
| `JointBatchExecutionStatus` | message | 批量关节命令最终状态 | 已实现 |
| `WorkspaceControl` / `WorkspaceStatus` | message | Workspace 生命周期和模式状态 | 已实现 |

核心接口约定：

- 关节角度使用弧度。
- TCP 平移使用米，姿态支持 RPY 和四元数语义。
- 速度和加速度由请求传入，并由高层规划器和门控逻辑检查。
- 服务 `success=true` 应表示最终闭环状态已经确认，不能只表示消息发布成功。
- `ArmPowerStatus::ENABLED_STATUS=39`、`DISABLED_STATUS=64` 是当前机器人配置中的状态码约定，迁移其他机器人时必须重新确认。

### 5.2 `robot_arm_description`：模型与可视化

位置：[`src/robot_arm_description`](/home/user/joint_controller/src/robot_arm_description)

已具备：

- 双臂 URDF、SRDF、Xacro 和 RViz 配置。
- 左右臂链、末端 link 和网格模型。
- `right_left_arm.urdf` 作为高层笛卡尔控制默认模型。
- `lee_link` 和 `ree_link` 作为默认左右末端坐标系。
- 机器人全身/底盘相关模型文件和独立显示启动入口。

待确认：

- 模型关节零位与真实机械零位的一致性。
- 左右末端坐标系原点、轴向和工具安装方向。
- 真实工具、夹具和负载的质量、质心及惯量参数。
- 碰撞模型、工作空间边界和现场安全区域是否与实物一致。

### 5.3 `erobot_controller`：机械臂 ros2_control 控制器

位置：[`src/erobot_controller`](/home/user/joint_controller/src/erobot_controller)

已具备：

- `ros2_control` 控制器插件。
- 轨迹执行相关的 `CommandProcessor_arm`、`CubicPlanner` 和 `traj` 代码。
- 机械臂控制器配置和初始位置配置。
- `load_controller_arm.launch.py`，支持 `sim`、RViz 和 EFFORT 模式切换参数。
- `joint_state_broadcaster_arm` 与 `erobot_controller_arm` 的启动顺序。
- 控制模式服务节点 `arm_control_mode_service`。

仿真启动时使用 `sim:=1.0`；真实栈使用真实硬件路径。POSITION 模式是笛卡尔和关节位置控制的前置条件，切回 POSITION 后需要新的位置目标，不应自动重放历史目标。

### 5.4 `erobot_hw`：机械臂硬件接口

位置：[`src/erobot_hw`](/home/user/joint_controller/src/erobot_hw)

已具备：

- 机械臂硬件接口插件和硬件描述 XML。
- 仿真与真实模式分支。
- 14 轴状态、位置、速度、力矩、模式、状态码和电源字段。
- 与现有 EtherCAT/共享内存控制数据结构的接口。

当前风险和待办：

- 包描述和代码风格检查仍有失败，见第 8 节。
- 共享内存对象、实际 EtherCAT owner、权限和重启清理必须在现场验证。
- 机械臂真实硬件通信、断链、异常状态和恢复流程不能仅由仿真结果替代。

### 5.5 `robot_control`：高层运动控制

位置：[`src/robot_control`](/home/user/joint_controller/src/robot_control)

启动入口：[`robot_control.launch.py`](/home/user/joint_controller/src/robot_control/launch/robot_control.launch.py)

`robot_control` 包内的主要可执行程序如下。默认 `robot_control.launch.py` 直接启动前六个控制/规划节点；Workspace supervisor 由独立的管理链路使用，不能据此认为它已经被该 launch 文件自动启动。

| 节点/可执行程序 | 主要职责 |
|---|---|
| `joint_absolute_control_srv` | 14 轴关节绝对控制服务，结合反馈等待运动完成 |
| `joint_batch_control_srv` | 选择性/批量关节命令、顺序和并发互斥 |
| `cartesian_single_control_srv` | 单点笛卡尔绝对/增量控制和最终 TCP 确认 |
| `cartesian_path_absolute_control_srv` | 多 waypoint 绝对路径服务 |
| `cartesian_path_increment_control_srv` | 多 waypoint 增量路径服务 |
| `cartesian_moveL_path` | 统一双臂 IK、路径采样、轨迹生成和执行状态发布 |
| `planner_input_publisher_ubuntu` | 规划输入兼容发布入口，按需要独立启动 |
| `workspace_supervisor_ubuntu` | Workspace SIM/REAL 启停和状态管理，通常由 systemd supervisor 链路使用 |

实现层面已经覆盖：

- 读取当前关节和 TCP 反馈。
- 检查 power、控制模式、反馈新鲜度和 planner busy 状态。
- 发布单点或路径目标。
- 等待 `STREAM_FINISHED`、运动停止和 TCP 到位。
- 记录规划失败阶段、左右臂目标/当前位姿、奇异性和关节限位相关诊断。
- 对输入的有限数、四元数有效性、范围和目标误差进行检查。
- 通过 callback group、条件变量和状态 generation 等机制等待异步反馈。

### 5.6 `joint_hardware`：升降与腰部外围硬件

位置：[`src/joint_hardware`](/home/user/joint_controller/src/joint_hardware)

#### 升降机构

已具备：

- EtherLab/EtherCAT backend 和 SDK adapter。
- PDO 映射、EtherCAT 帧编解码和 CoE/SDO 支持。
- CiA 402 状态机、故障复位、Quick Stop 和安全失能路径。
- 编码器、丝杆导程、方向、米/编码器单位换算。
- 软限位、可选物理限位输入、制动器控制和制动延时参数。
- 零偏存储和恢复。
- Ruckig 运动 profile、键盘遥操作和 mock 硬件集成路径。
- `lift_ethercat.launch.py`，支持 mock 和 etherlab backend。

当前默认/配置重点：

- `cycle_ms=10`。
- 默认 EtherCAT master index 为 `2`，机械臂 master 预留 `0/1`。
- 默认位置范围记录为 `-1.0 m` 到 `0.0 m`，现场仍需依据机械极限复核。
- `brake_control_enabled` 和 `limit_switch_enabled` 默认关闭，真机启用前必须完成独立确认。

#### 腰部机构

已具备：

- USB-FDCAN bridge hub。
- CAN-FD/BRS 同步帧、设备初始化、状态读取和命令写入。
- 清错服务、置零服务和反馈状态发布。
- 腰部 ros2_control 硬件接口和独立 launch。

待验证：

- USB 设备路径、CAN channel、node id、bitrate 和实际设备参数。
- 真实腰部方向、零位、限位、故障恢复和断开重连。
- 腰部运动与双臂/升降同时动作时的整机协调性。

### 5.7 `robot_lower_gateway`：可移植下位机网关

位置：[`src/robot_lower_gateway`](/home/user/joint_controller/src/robot_lower_gateway)

第一阶段已经形成的能力：

- 持续读取原控制栈反馈，不轮询调用控制 service。
- 原生关节名到固定 14 轴 canonical 顺序的映射。
- 电源、模式、运动、TCP 和 Cartesian execution 状态归一化。
- 关节反馈完整性、NaN/Inf 和 stale 检查。
- `diagnostic_msgs/msg/DiagnosticArray` 健康诊断。
- pluginlib backend 结构和 `erobot_v1.yaml` profile。
- 默认只读，默认不提供全局控制服务。
- 可显式启用 `/ubuntu_lower_gateway/*` 命名空间 command proxy。

默认输出：

| 输出 | 类型 | 当前约定 |
|---|---|---|
| `/ubuntu_lower_gateway/joint_states` | `sensor_msgs/msg/JointState` | 默认 20 Hz，输入完整且新鲜时发布 |
| `/ubuntu_lower_gateway/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 汇总反馈新鲜度、轴状态、模式和 proxy 状态 |

proxy 目前转发的原生服务包括电源、模式、批量关节、关节绝对、笛卡尔增量和笛卡尔绝对服务。proxy 默认关闭，正式启用前必须完成仿真 A/B、失败透传和 owner 唯一性验证。

## 6. 启动、部署和日常操作进度

### 6.1 仿真启动

仿真主入口：

```bash
cd /home/user/joint_controller
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=55
./scripts/run_joint_controller_stack_sim.sh
```

或直接使用：

```bash
ros2 launch robot_control joint_controller_stack_sim.launch.py
```

仿真启动链路为：

1. 启动 `erobot_controller` 的 `ros2_control_node`。
2. 以 `sim:=1.0` 加载机器人描述和仿真硬件。
3. 启动 joint state broadcaster 和机械臂 controller。
4. 启动控制模式服务。
5. 启动 `robot_control` 高层服务和统一 MoveL 节点。
6. 通过 supervisor 或脚本统一管理 SIM stack 和 RViz。

### 6.2 真实栈启动

真实栈由 `joint-controller-stack-real.service` 管理。启动真实栈前必须完成：

- EtherCAT 网口、master、slave、PDO 和 working counter 检查。
- 急停、失能和制动器状态确认。
- 机械臂和外围硬件的真实反馈确认。
- 仿真 unit 已停止，避免 SIM/REAL 同时运行。
- 上位机和下位机 DDS 参数一致。
- 真实电源和真实运动开关保持默认关闭，直到完成现场安全确认。

建议采用项目现有的顺序：

```text
up real -> start real -> status -> 检查反馈 -> mode position -> power on
```

停止顺序：

```text
power off -> stop -> down
```

发生非预期运动时，物理急停优先于软件命令。

### 6.3 systemd 组件

当前目录提供：

| unit | 作用 |
|---|---|
| `joint-controller-supervisor.service` | Workspace supervisor 常驻节点 |
| `joint-controller-stack-sim.service` | 仿真控制栈 |
| `joint-controller-stack-real.service` | 真实控制栈 |
| `joint-controller-stack.service` | 兼容/旧版控制栈入口 |
| `joint-controller-rviz.service` | 仿真 RViz |

systemd 配置中已经设置了 SIM/REAL unit 的冲突关系。安装或更新后应检查：

- 只有期望的 supervisor 自动运行。
- SIM 和 REAL stack 不会同时 active。
- unit 使用的是当前工作空间的脚本和 `install/setup.bash`。
- 进程退出后状态能够被 supervisor 识别。
- 重启后不自动绕过安全状态打开电机。

### 6.4 workflow 自动化

入口：[`scripts/run_workflow.py`](/home/user/joint_controller/scripts/run_workflow.py)

已有 workflow：

| 文件 | 用途 |
|---|---|
| `smoke_sim_workflow.yaml` | 仿真冒烟流程 |
| `emergency_stop_test.yaml` | 软件安全停止流程 |
| `lift_hold_lower_cycle.yaml` | 升降保持和下降循环 |
| `example_box_workflow.yaml` | 箱体相关双臂任务示例 |
| `example_box_workflow.json` | JSON 示例任务 |

workflow 已支持：

- 姿态库和 `l1`~`l7`、`r1`~`r7` 字段。
- 单个关节、左右对应关节和关节组顺序执行。
- 笛卡尔增量动作。
- `confirm_after`、`--confirm-all` 和 `--no-confirm`。
- 一次 Ctrl+C 停止后续动作，连续两次尝试关闭机器人电源。
- `--dry-run` 只打印动作，不发布控制命令。

## 7. 接口和安全闭环进度

### 7.1 电源控制闭环

当前设计要求 `/set_robot_power`：

1. 校验请求和当前反馈结构。
2. 通过唯一的 Ubuntu service owner 下发命令。
3. 等待 service 最终响应。
4. 继续等待新的 `/arm/power_status`。
5. 只有目标状态确认后才返回 `success=true`。

当前记录中的状态码：

- 使能：14/14 电机状态码 `39`。
- 失能：14/14 电机状态码 `64`。

待补充：实际状态更新时间、掉线时行为、单电机故障、急停触发后的状态，以及电源失能到机械停止的时间。

### 7.2 控制模式闭环

当前支持：

- `POSITION=0`。
- `EFFORT=1`，是否允许切换受 `enable_effort_mode_switch` 和上位机安全参数控制。
- 状态反馈包括 `requested_mode`、`active_mode` 和 `position_command_ready`。
- 切回 POSITION 后，不自动重发旧的位置目标。
- 模式切换期间应拒绝冲突的运动和电源请求。

### 7.3 笛卡尔控制闭环

单点绝对/增量接口的闭环条件包括：

- workspace 处于 RUNNING，模式明确为 SIMULATION 或 REAL。
- 电机反馈合法且新鲜，位置控制所需电机状态满足要求。
- 控制模式为 POSITION，且位置命令就绪。
- TCP、motion、Cartesian execution 反馈均已收到且未过期。
- planner 不处于 PLANNING 或 EXECUTING。
- 同时没有其他 joint、Cartesian、power 或 mode 命令执行。
- 请求数值有限，速度、加速度、位移和姿态增量不超过安全限制。
- 等待路径流结束、运动停止和 TCP 目标误差确认。

现有项目记录的仿真数据：

| 动作 | 项目文档记录值 | 本文处理方式 |
|---|---:|---|
| 左臂 X 增量 5 mm 最终位置误差 | 约 `0.000408 m` | 作为已有记录，后续需用统一脚本复测并补充时间、姿态和左右臂结果 |
| 绝对回位最终位置误差 | 约 `0.000032 m` | 作为已有记录，后续需补充测试条件和重复次数 |

以上数值不是整机验收结论，也不代替真实机器人结果。

## 8. 测试与验证证据

### 8.1 自动化测试覆盖

`joint_hardware` 已配置以下 GTest：

| 测试 | 覆盖内容 | 当前记录 |
|---|---|---|
| `test_lift_units` | 升降单位、方向和位置换算 | 通过 |
| `test_lift_motion_profile` | 升降运动 profile | 通过 |
| `test_lift_controller` | 升降 controller 行为 | 通过 |
| `test_cia402` | CiA 402 状态机 | 通过 |
| `test_zero_offset` | 零偏存储和处理 | 通过 |
| `test_mock_lift_integration` | mock 升降集成 | 通过 |
| `test_ethercat_frames` | EtherCAT 帧编解码 | 通过 |
| `test_waist_can` | 腰部 CAN 协议 | 通过 |
| `test_usb_bridge_hub` | USB-FDCAN 桥接 | 通过 |

`robot_lower_gateway` 已配置：

| 测试 | 覆盖内容 | 当前记录 |
|---|---|---|
| `test_joint_state_mapping` | 原生关节名、固定 14 轴顺序和数据映射 | 通过 |
| `test_feedback_policy` | stale、完整性、终态和诊断策略 | 通过 |
| `test_native_service_forwarder` | 原生 service 转发、超时和响应透传 | 通过 |

### 8.2 当前工作空间测试结果摘要

本次读取工作空间已有的 `colcon test-result --all` 结果，摘要为：

| 项目 | 数量 |
|---|---:|
| 测试总数 | 148 |
| errors | 0 |
| failures | 11 |
| skipped | 56 |

当前可明确定位到的失败主要包括：

- `erobot_hw` 的 3 个 `uncrustify` 代码风格检查失败。
- `erobot_hw/package.xml` 的 `xmllint` 检查失败，测试环境无法加载 ROS package schema。
- `robot_control_msg` 的 2 个 `uncrustify` 代码风格检查失败。
- `robot_control_msg/package.xml` 的 `xmllint` 检查失败，同样涉及外部 ROS schema 加载。

需要注意：

- 这些结果说明当前测试报告不是“全绿”，不能在项目汇报中写成所有测试通过。
- 已通过的 GTest 说明对应模块逻辑测试通过，不代表 EtherCAT、CAN 或整机运动已验收。
- `xmllint` 的 schema 失败带有环境/网络因素，但仍应在有 schema 的环境中重新执行。
- `uncrustify` 失败属于可修复的代码风格问题，应单独清理后复测。

### 8.3 已有仿真和部署验证记录

现有项目文档记录了以下已完成或已验证方向：

- Workspace SIM/REAL 生命周期及 supervisor 逻辑。
- 电源服务最终状态确认。
- POSITION/EFFORT 模式切换闭环。
- 单关节/14 轴控制。
- 笛卡尔单点控制和 TCP 误差确认。
- 反馈网关第一阶段适配和诊断策略。

这些内容在正式报告中建议标记为“已有仿真/软件验证记录”，直到现场执行日志、测试日期、设备编号和原始数据归档后，再升级为“现场验证通过”。

### 8.4 近期完成工作

近期围绕抓箱子任务和整机安装完成以下工作：

- 完成腰部电机和升降电机的负压测试；测试结束后测量电机温度，温度状态良好，未发现异常发热。具体温度值待补录。
- 完成重型机器人底盘结构改良，优化整机重心位置和承重能力。
- 完成整机线束整理和拖链安装，改善线束固定、防护及运动过程中的随动性。
- 后续结合抓箱子整机联动测试，继续确认线束干涉、拖链弯曲半径、结构稳定性和长时间运行温升。

## 9. 整机联调与结果记录（待填写）

> 本节按用户要求保留为空。现场测试时请填写测试日期、设备编号、代码版本、操作者、环境条件、原始日志位置和最终结论。

### 9.1 整机基本信息

| 项目 | 结果 |
|---|---|
| 机器人型号/编号 |  |
| 控制柜编号 |  |
| 上位机编号 |  |
| 下位机编号 |  |
| 软件版本/提交号 |  |
| 测试日期 |  |
| 测试地点 |  |
| 测试人员 |  |
| 机械臂负载配置 |  |
| 末端工具/夹具 |  |
| 环境温度 |  |
| 测试前机械状态 |  |

### 9.2 整机启动和停止结果

| 测试项 | 预期结果 | 实测结果 | 是否通过 | 备注/日志 |
|---|---|---|---|---|
| 开机后 supervisor 状态 | 仅 supervisor 运行，工作空间不自动运动 |  |  |  |
| 启动 SIM workspace | `RUNNING + SIMULATION` |  |  |  |
| 停止 SIM workspace | 安全停止，SIM unit inactive |  |  |  |
| 启动 REAL workspace | `RUNNING + REAL`，硬件无异常 |  |  |  |
| SIM/REAL 互斥 | 不同时运行 |  |  |  |
| `power off -> stop -> down` | 0/14 电机使能后再停止 |  |  |  |
| 异常退出恢复 | 状态进入 ERROR 或可控停止，不自动危险重启 |  |  |  |
| 重启后状态 | 不自动使能电机，不自动执行旧目标 |  |  |  |

### 9.3 整机电源与控制模式结果

| 测试项 | 预期结果 | 实测结果 | 是否通过 | 备注 |
|---|---|---|---|---|
| 失能初始状态 | 0/14 使能，状态码符合配置 |  |  |  |
| SIM 电机使能 | 14/14 状态码 39，最终响应为成功 |  |  |  |
| SIM 电机失能 | 0/14 使能，最终响应为成功 |  |  |  |
| REAL 电机使能 | 14/14 使能，无报警和异常发热 |  |  |  |
| REAL 电机失能 | 0/14 使能 |  |  |  |
| POSITION 模式切换 | `active_mode=POSITION` |  |  |  |
| EFFORT 模式切换 | 仅在明确允许时成功 |  |  |  |
| 断开状态反馈 | 请求被拒绝或返回失败，不伪造成功 |  |  |  |
| 单电机异常 | 不能误报 14/14 全部正常 |  |  |  |
| 急停后软件状态 | 电机停止/失能状态与现场一致 |  |  |  |

### 9.4 整机 14 轴关节控制结果

| 测试项 | 实测位置误差 | 运动时间 | 重复次数 | 成功次数 | 是否通过 | 备注 |
|---|---:|---:|---:|---:|---|---|
| 左臂逐轴低速正向 |  |  |  |  |  |  |
| 左臂逐轴低速反向 |  |  |  |  |  |  |
| 右臂逐轴低速正向 |  |  |  |  |  |  |
| 右臂逐轴低速反向 |  |  |  |  |  |  |
| 双臂同步小幅动作 |  |  |  |  |  |  |
| 14 轴全量目标 |  |  |  |  |  |  |
| 关节边界附近动作 |  |  |  |  |  |  |
| 目标超限拒绝 |  |  |  |  |  |  |
| 运动中重复命令拒绝 |  |  |  |  |  |  |

### 9.5 整机笛卡尔控制结果

| 测试项 | 左臂位置误差 | 右臂位置误差 | 左臂姿态误差 | 右臂姿态误差 | 是否通过 | 备注 |
|---|---:|---:|---:|---:|---|---|
| 左臂 X 方向增量 |  |  |  |  |  |  |
| 右臂 X 方向增量 |  |  |  |  |  |  |
| 双臂相向增量 |  |  |  |  |  |  |
| 双臂同步上升 |  |  |  |  |  |  |
| 单臂绝对回位 |  |  |  |  |  |  |
| 双臂绝对回位 |  |  |  |  |  |  |
| RPY 姿态控制 |  |  |  |  |  |  |
| 四元数姿态控制 |  |  |  |  |  |  |
| 多路点绝对路径 |  |  |  |  |  |  |
| 多路点增量路径 |  |  |  |  |  |  |
| 奇异位形拒绝/退出 |  |  |  |  |  |  |
| 关节限位拒绝/退出 |  |  |  |  |  |  |

### 9.6 整机升降、腰部和双臂协同结果

| 测试项 | 实测结果 | 是否通过 | 备注 |
|---|---|---|---|
| 升降上升 |  |  |  |
| 升降下降 |  |  |  |
| 升降软限位 |  |  |  |
| 升降物理限位 |  |  |  |
| 升降制动器 |  |  |  |
| 升降零位保持 |  |  |  |
| 腰部正向动作 |  |  |  |
| 腰部反向动作 |  |  |  |
| 腰部清错 |  |  |  |
| 腰部置零 |  |  |  |
| 腰部断线恢复 |  |  |  |
| 双臂 + 升降协同 |  |  |  |
| 双臂 + 腰部协同 |  |  |  |
| 双臂 + 升降 + 腰部协同 |  |  |  |

## 10. 整机性能数据（待填写）

### 10.1 实时性和通信

| 指标 | 目标值 | 实测值 | 测试条件 | 是否通过 |
|---|---:|---:|---|---|
| 关节状态发布频率 |  |  |  |  |
| TCP 状态发布频率 |  |  |  |  |
| motion status 发布频率 |  |  |  |  |
| execution status 发布频率 |  |  |  |  |
| 控制命令端到端延迟 |  |  |  |  |
| 电源命令确认时间 |  |  |  |  |
| 模式切换确认时间 |  |  |  |  |
| TCP 单点动作耗时 |  |  |  |  |
| DDS 丢包/重传情况 |  |  |  |  |
| EtherCAT Tx errors |  |  |  |  |
| EtherCAT lost frames |  |  |  |  |

### 10.2 精度、重复性和稳定性

| 指标 | 目标值 | 实测值 | 样本数 | 是否通过 | 备注 |
|---|---:|---:|---:|---|---|
| 左臂关节定位精度 |  |  |  |  |  |
| 右臂关节定位精度 |  |  |  |  |  |
| 左臂 TCP 位置精度 |  |  |  |  |  |
| 右臂 TCP 位置精度 |  |  |  |  |  |
| 左臂 TCP 姿态精度 |  |  |  |  |  |
| 右臂 TCP 姿态精度 |  |  |  |  |  |
| 双臂相对位置误差 |  |  |  |  |  |
| 同一目标重复定位误差 |  |  |  |  |  |
| 连续运行时长 |  |  |  |  |  |
| 连续运行期间异常次数 |  |  |  |  |  |
| 电机/控制柜温升 |  |  |  |  |  |

### 10.3 负载和任务性能

| 指标 | 目标值 | 实测值 | 负载条件 | 是否通过 |
|---|---:|---:|---|---|
| 最大安全搬运质量 |  |  |  |  |
| 箱体抓取成功率 |  |  |  |  |
| 箱体抬升成功率 |  |  |  |  |
| 箱体保持时长 |  |  |  |  |
| 多次搬运循环次数 |  |  |  |  |
| 任务平均耗时 |  |  |  |  |
| 任务失败原因统计 |  |  |  |  |

## 11. 安全、异常和故障注入结果（待填写）

| 场景 | 预期行为 | 实测行为 | 是否通过 | 备注 |
|---|---|---|---|---|
| 急停触发 | 运动停止，后续不能继续危险动作 |  |  |  |
| 失能命令 | 尽力执行失能并返回真实结果 |  |  |  |
| workspace STOP 中请求运动 | 拒绝 |  |  |  |
| workspace mode UNKNOWN 中请求运动 | 拒绝 |  |  |  |
| power 反馈 stale | 拒绝使能/运动 |  |  |  |
| TCP 反馈 stale | 拒绝笛卡尔运动或返回失败 |  |  |  |
| motion/execution 反馈 stale | 不伪造完成 |  |  |  |
| 四元数非法 | 拒绝请求 |  |  |  |
| NaN/Inf 请求 | 拒绝请求 |  |  |  |
| 超出关节限位 | 拒绝或安全退出 |  |  |  |
| 超出笛卡尔位移上限 | 拒绝请求 |  |  |  |
| planner busy 时重复请求 | 拒绝并保持当前动作 |  |  |  |
| 控制器进程异常退出 | workspace 进入可识别异常状态 |  |  |  |
| DDS 断开 | 状态 stale，不能继续误动作 |  |  |  |
| EtherCAT 从站掉线 | 进入故障处理/停止路径 |  |  |  |
| 腰部 USB-CAN 断开 | 报错并禁止伪造反馈 |  |  |  |
| 升降 PDO 交换失败 | 进入 transport fault 和安全停止 |  |  |  |

## 12. 当前问题、风险和待办

### 12.1 已确认的问题

1. 当前已有 `colcon test-result` 汇总不是全通过，存在 11 个 failure。
2. `erobot_hw` 和 `robot_control_msg` 存在 `uncrustify` 风格检查失败。
3. 部分 `package.xml` 的 `xmllint` 检查依赖外部 ROS schema，当前测试环境无法加载 schema。
4. 代码仓库中仍存在多个 `0.0.0` 版本和 `TODO` 描述/许可证字段，发布和交付前需要统一。
5. 机械臂高层服务已有较多异常和闭环逻辑，但针对整机的自动化测试尚不充分。
6. 真实硬件测试结果、整机任务成功率、负载和长期稳定性数据尚未形成正式归档。

### 12.2 需要重点关注的技术风险

- 真实硬件与仿真模型的关节方向、零位、限位和末端坐标系可能不一致。
- `39/64` 等电机状态码是机器人相关约定，迁移设备时不能直接复用。
- 升降、腰部和机械臂分别存在不同通信 owner，整机同时动作时需要确认资源和安全互锁。
- systemd、手工启动和旧版兼容 unit 若同时存在，可能造成重复节点或服务 owner 冲突。
- gateway proxy 虽然默认关闭，但启用时仍必须证明不会产生第二个全局 service owner。
- 运动服务等待异步反馈，线程、callback group、超时和 busy 清理路径需要持续做压力测试。
- 真实场景中断电、网络断开、EtherCAT 丢帧、USB-CAN 断开和急停恢复不能用普通 service 超时简单代替。

### 12.3 建议优先级

| 优先级 | 事项 | 完成标准 |
|---|---|---|
| P0 | 完成整机安全链路确认 | 急停、失能、workspace stop 和恢复行为有现场记录 |
| P0 | 完成 REAL 硬件基础验收 | EtherCAT/CAN/反馈/零位/模式/电源均有日志 |
| P1 | 清理测试失败 | 重新跑 lint、schema 和 GTest，保留测试报告 |
| P1 | 复测仿真控制精度 | 脚本、参数、重复次数和原始反馈统一归档 |
| P1 | 完成 gateway 仿真 A/B | 只读、proxy、超时、失败和 owner 唯一性全部通过 |
| P2 | 增加高层控制自动化测试 | 关节、笛卡尔、路径和异常输入具备可重复测试 |
| P2 | 完成整机任务测试 | 箱体抓取、抬升、保持、下降和失败统计有结果 |
| P3 | 工程发布整理 | 版本号、许可证、README、目录和部署包统一 |

## 13. 后续开发计划

### 阶段 A：软件质量收口

- 修复 `erobot_hw` 和 `robot_control_msg` 的 uncrustify 差异。
- 在可访问 ROS schema 的环境重跑 xmllint。
- 检查所有包的 `package.xml` 描述、license、maintainer 和 version。
- 固定一份可复现的构建命令、ROS 环境和测试报告归档方式。
- 清理或明确旧版 systemd unit、兼容 topic 和历史命名。

### 阶段 B：仿真回归

- 从空 workspace 启动仿真。
- 依次验证模式、power、关节、笛卡尔单点和多路点路径。
- 验证所有默认安全开关为关闭状态。
- 验证 stale、busy、超限、非法四元数和中断路径。
- 重复记录 5 mm 增量、绝对回位和双臂同步动作的误差。
- 执行 workflow 和 `safe-stop`，确认结束状态干净。

### 阶段 C：真实硬件基础联调

- 确认 EtherCAT master、网口、从站顺序、PDO、working counter 和时钟。
- 保持电机失能，先只读确认关节、电源、模式、TCP 和 execution 反馈。
- 确认机械零位、方向、软限位、物理限位和制动器。
- 单轴低速动作，再进行单臂、双臂和外围机构动作。
- 每一步都保存终端日志、topic 快照、service 回执和急停状态。

### 阶段 D：整机任务和性能

- 按第 9 至第 11 节填写完整结果。
- 记录定位精度、重复性、运动时间、温升、丢帧和异常次数。
- 使用固定工具、固定负载、固定姿态和固定测试次数。
- 对失败任务记录动作阶段、错误消息、反馈状态和恢复动作。
- 形成“软件版本 + 机器人编号 + 测试数据 + 原始日志”的闭环档案。

### 阶段 E：部署和移植交付

- 完成 Jetson/Ubuntu 两端网络、DDS、版本和启动脚本固化。
- 完成独立 gateway overlay 的安装、systemd 和回滚流程。
- 仅在 proxy 仿真 A/B 和现场门槛满足后启用 command proxy。
- 对新机器人重新确认关节映射、状态码、topic、QoS、模式枚举和超时参数。

## 14. 参考文件索引

| 内容 | 文件 |
|---|---|
| 项目概览和控制接口示例 | [`README.md`](../README.md) |
| 现有关节姿态和笛卡尔命令记录 | [`joint_state.md`](../joint_state.md) |
| workflow 使用说明 | [`workflows/README.md`](../workflows/README.md) |
| 上下位机部署入口 | [`robot_system_deployment.md`](robot_system_deployment.md) |
| Ubuntu 下位机部署和验收 | [`ubuntu_lower_machine_deployment.md`](ubuntu_lower_machine_deployment.md) |
| Jetson 上位机部署 | [`jetson_upper_machine_deployment.md`](jetson_upper_machine_deployment.md) |
| gateway 部署 | [`robot_lower_gateway_deployment.md`](robot_lower_gateway_deployment.md) |
| gateway 迁移开发说明 | [`lower_gateway_migration.md`](lower_gateway_migration.md) |
| gateway 功能包说明 | [`../src/robot_lower_gateway/README.md`](../src/robot_lower_gateway/README.md) |
| 升降/腰部硬件说明 | [`../src/joint_hardware/README.md`](../src/joint_hardware/README.md) |
| 仿真控制启动 | [`../src/robot_control/launch/joint_controller_stack_sim.launch.py`](../src/robot_control/launch/joint_controller_stack_sim.launch.py) |
| 高层控制启动 | [`../src/robot_control/launch/robot_control.launch.py`](../src/robot_control/launch/robot_control.launch.py) |
| 下位机网关配置 | [`../src/robot_lower_gateway/config/erobot_v1.yaml`](../src/robot_lower_gateway/config/erobot_v1.yaml) |

## 15. 阶段结论

当前工作空间已经从单一机械臂控制代码扩展为包含模型、仿真、真实硬件接口、高层运动规划、上下位机部署、workspace 管理、外围机构和可移植反馈网关的完整软件体系。关节控制、笛卡尔控制、闭环状态、systemd 管理和模块级测试均已有实现基础。

当前不宜直接宣称“整机开发完成”或“整机验收通过”。更准确的阶段结论是：

> 控制软件主链路和部署框架已基本形成；部分仿真/模块验证已有记录；当前主要工作转入测试质量收口、真实硬件基础联调、整机安全验收、任务性能测试和结果归档阶段。

整机最终结论、精度、负载、成功率、连续运行时间和安全测试结果，统一以本文第 9、10、11 节现场填写内容为准。
