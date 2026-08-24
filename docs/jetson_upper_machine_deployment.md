# Jetson 上位机部署与操作

## 1. 固定环境

| 项目 | 值 |
|---|---|
| Jetson 控制网 IP | `192.168.2.10/24` |
| Ubuntu 下位机 IP | `192.168.2.20/24` |
| 工作空间 | `/home/yuling/robot_gateway_ws` |
| ROS 2 | Humble |
| ROS Domain | `55` |
| RMW | `rmw_fastrtps_cpp` |

旧工作空间 `/home/yuling/joint_controller` 只用于历史版本。新部署和日常操作统一使用 `/home/yuling/robot_gateway_ws`，不要同时启动旧 `planner_node_jetson`。

## 2. 配置控制网口

查看真实接口名和 NetworkManager 连接名：

```bash
ip -br link
ip -br addr
nmcli device status
nmcli connection show
```

下面假设 Jetson 控制网口为 `eno1`，现场必须替换为真实名称：

```bash
sudo nmcli connection add \
  type ethernet \
  ifname eno1 \
  con-name robot-network \
  ipv4.method manual \
  ipv4.addresses 192.168.2.10/24 \
  ipv4.never-default yes \
  ipv6.method disabled

sudo nmcli connection up robot-network
```

验证：

```bash
ip -4 addr show dev eno1
ip route
ethtool eno1
ping -c 4 192.168.2.20
ping -M do -s 1472 -c 10 192.168.2.20
```

要求接口为 `UP`、`Link detected: yes`、包含 `MULTICAST`，两端 MTU 都是 `1500`。UFW 已启用时只放行机器人子网，不要直接关闭整机防火墙：

```bash
sudo ufw allow from 192.168.2.0/24
sudo ufw allow to 192.168.2.0/24
```

## 3. 安装依赖与编译

```bash
cd /home/yuling/robot_gateway_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

工作空间应包含 Jetson 侧功能包：

```text
src/
├── robot_control_msg
├── robot_lower_gateway
├── robot_gateway_client
├── robotctl_cli
└── robot_gateway_bringup
```

Jetson 与 Ubuntu 的 `robot_control_msg` 必须是同一版本。发布部署包时建议保存接口文件 SHA-256 并在两端对比。

## 4. 终端环境

每个新终端执行：

```bash
cd /home/yuling/robot_gateway_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=55
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset ROS_DISCOVERY_SERVER
unset CYCLONEDDS_URI
unset FASTRTPS_DEFAULT_PROFILES_FILE
```

注意命令是 `source`，不是 `souce`。安装后的推荐入口是 `robotctl`；找不到命令时先检查：

```bash
export PATH="$HOME/.local/bin:$PATH"
command -v robotctl
```

未安装命令包装时，`robotctl` 等价于：

```bash
ros2 run robotctl_cli robotctl
```

## 5. 命令语义

```text
robotctl up sim/real    只启动 Jetson gateway/planner profile
robotctl start sim/real 请求 Ubuntu 启动对应 workspace
robotctl stop           先失能并停止 Ubuntu workspace
robotctl down           只停止 Jetson gateway/planner
robotctl status         汇总上下位机反馈
```

禁止把 `up` 当成 Ubuntu workspace 已启动，也禁止用 `down` 代替 `power off -> stop`。

## 6. 仿真流程

一键准备：

```bash
robotctl ready sim
robotctl status
```

应看到：

```text
workspace=RUNNING mode=SIMULATION
power=14/14 all_enabled=True
control_mode=POSITION
joint_states=14 joints
```

也可以逐步执行：

```bash
robotctl down
robotctl up sim
robotctl start sim
robotctl mode position
robotctl power on
robotctl status
```

仿真停止：

```bash
robotctl power off
robotctl stop
robotctl down
```

## 7. 常用控制

单关节绝对目标，单位为弧度：

```bash
robotctl joint ljoint6 0.10 --vel 0.10 --acc 0.10
```

单关节相对增量：

```bash
robotctl joint ljoint6 0.01 --relative --vel 0.10 --acc 0.10
```

14 轴交互编辑：

```bash
robotctl joint edit
```

笛卡尔小增量：

```bash
robotctl cart-inc left --x 0.005 --vel 0.05 --acc 0.10
robotctl cart-inc right --z -0.003 --vel 0.05 --acc 0.10
```

保存和恢复双臂 TCP 位姿：

```bash
robotctl pose save pose_1
robotctl cart-abs pose_1 --vel 0.05 --acc 0.10
```

只读监控：

```bash
robotctl watch joints --once
robotctl watch joints --rate 20
robotctl watch joints --rate 20 --degrees
```

## 8. 真机流程

真机禁止使用无确认的一键 ready。急停必须可用，机械臂周围无人，第一次测试从失能状态和极小增量开始。

```bash
robotctl down
robotctl up real --confirm-real
robotctl start real --confirm-real
robotctl status
robotctl power off
robotctl mode position
robotctl watch joints --once
```

确认以下条件后才允许使能：

```text
workspace=RUNNING mode=REAL
joint_states=14 joints，且全部为有限值
control_mode=POSITION
power=0/14
```

只执行一次使能：

```bash
robotctl power on
robotctl status
```

必须看到 `power=14/14 all_enabled=True`。否则禁止运动。

第一次单关节测试：

```bash
robotctl joint ljoint7 0.005 --relative --vel 0.02 --acc 0.02
```

第一次笛卡尔测试最多使用约 `1 mm`：

```bash
robotctl pose save real_deployment_baseline
robotctl cart-inc left --x 0.001 --vel 0.01 --acc 0.02
robotctl cart-abs real_deployment_baseline --vel 0.01 --acc 0.02
```

每条命令必须等待最终返回，不得并发发送。正常停止：

```bash
robotctl power off
robotctl stop
robotctl down
```

## 9. 状态与排障

```bash
robotctl status
robotctl logs
ros2 node list --no-daemon --spin-time 5
ros2 service list | rg 'workspace|planner|telemetry'
ros2 param list /robot_lower_gateway
```

常见含义：

- `service unavailable: /workspace/request_start_real`：Jetson real profile 没有启动或环境不一致。
- `workspace=STOPPED mode=UNKNOWN`：Ubuntu supervisor 存在，但控制栈尚未启动。
- `power=WAITING`：Ubuntu 控制栈未运行或 DDS 不通。
- `cartesian_servers=0`：Ubuntu `robot_control` 启动不完整。
- `power off` 超时：可能已经先停止 workspace，电源服务已经退出。

`/robot_lower_gateway` 是 Jetson 控制节点。Ubuntu 新增的可移植只读节点是 `/ubuntu_lower_gateway`，不要混用两者的参数和日志。
