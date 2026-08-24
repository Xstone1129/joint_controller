# 双臂机器人上下位机部署入口

本文是整套系统文档入口。不要把 Jetson 操作、Ubuntu/EtherCAT 部署和可移植网关开发混写在一个连续命令列表中。

## 1. 系统角色

| 设备 | 固定地址 | 工作空间 | 职责 |
|---|---|---|---|
| Jetson 上位机 | `192.168.2.10/24` | `/home/yuling/robot_gateway_ws` | `robotctl`、远程命令、安全门控和结果交叉确认 |
| Ubuntu 下位机 | `192.168.2.20/24` | `/home/user/joint_controller` | ros2_control、IK、EtherCAT、真实执行和最终安全检查 |

两端统一使用：

```text
ROS_DOMAIN_ID=55
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

## 2. 文档导航

1. [Jetson 上位机部署与操作](jetson_upper_machine_deployment.md)
2. [Ubuntu 下位机部署、自启动与验收](ubuntu_lower_machine_deployment.md)
3. [下位机通信功能包完整部署与迁移](robot_lower_gateway_deployment.md)
4. [可移植下位机网关开发说明](lower_gateway_migration.md)

首次部署按 1、2 的环境章节分别完成两台机器配置，再执行 Ubuntu 开机监听验收，最后从 Jetson 进行 SIM/REAL 测试。迁移到其他同构机器人时再阅读第 3 份文档。

## 3. 两个 gateway 的区别

当前系统存在两个不同用途的节点，名称必须区分：

| 位置 | 节点 | 作用 |
|---|---|---|
| Jetson | `/robot_lower_gateway` | 对外提供 workspace、电源、模式、关节和笛卡尔控制入口 |
| Ubuntu | `/ubuntu_lower_gateway` | 可移植反馈适配器；默认只读，可选命名空间 command proxy |

Ubuntu 的 `robot_lower_gateway` 是 ROS package 名；其运行节点故意命名为 `/ubuntu_lower_gateway`，避免与 Jetson 节点冲突。

## 4. 当前实现状态

| 能力 | 当前 owner | 状态 |
|---|---|---|
| Workspace SIM/REAL | Jetson gateway + Ubuntu supervisor | 已验证 |
| 电机使能/失能 | Jetson gateway + Ubuntu controller | 已验证 |
| 控制模式 | Jetson gateway + Ubuntu controller | 已验证 |
| 单关节/14 轴控制 | Jetson gateway + Ubuntu `robot_control` | 已验证 |
| 笛卡尔控制 | Jetson gateway + Ubuntu `robot_control` | 已验证 |
| 可移植反馈适配 | Ubuntu `/ubuntu_lower_gateway` | 第一阶段已实现，需现场 DDS 验收 |
| 可移植网关 command proxy | `/ubuntu_lower_gateway/*` -> 原 Ubuntu service | 已实现，默认关闭，需仿真 A/B 验收 |

现有全局 `/set_robot_power`、模式、关节和笛卡尔服务仍由原 Ubuntu 工作空间提供。网关
proxy 只提供 `/ubuntu_lower_gateway/*` 命名空间转发，不直接写硬件，也不改变全局 service
owner。

## 5. 标准部署顺序

1. 配置两端控制网口并相互 `ping`。
2. 安装相同 ROS 2 Humble、RMW 和 `robot_control_msg` 接口版本。
3. 编译 Ubuntu 原控制工作空间并安装 systemd units。
4. 确认 Ubuntu 重启后只有 supervisor 自动运行，workspace 为 `STOPPED/UNKNOWN`。
5. 编译 Jetson gateway 工作空间。
6. 先执行 SIM 验收和安全停止。
7. 真机检查 EtherCAT、急停和失能状态后，逐步执行 REAL 验收。
8. 最后旁路启动 `/ubuntu_lower_gateway`，验证规范化反馈，不改变原控制 owner。

## 6. 停止顺序

正常停止始终使用：

```bash
robotctl power off
robotctl stop
robotctl down
```

`robotctl down` 只停止 Jetson gateway/planner，不能代替电机失能和 Ubuntu workspace 停止。发生非预期运动时优先使用物理急停。
