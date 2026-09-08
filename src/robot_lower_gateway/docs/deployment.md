# robot_lower_gateway 下位机通信功能包部署与迁移手册

本文是 `robot_lower_gateway` 的统一部署文档，覆盖控制网络、ROS 2/DDS 环境、功能包配置、
开机自启动、上下位机联调，以及迁移到其他机器人工作空间或更换上位机电脑的流程。

本文当前基线如下。迁移时先建立一张新设备参数表，再替换示例值，不要直接复制网口名或
MAC 地址。

| 项目 | 当前值 |
|---|---|
| Ubuntu 下位机 | `192.168.2.20/24` |
| 下位机控制网口 | `enp8s0` |
| 当前下位机工作空间 | `/home/user/joint_controller` |
| Jetson 上位机 | `192.168.2.10/24` |
| ROS 2 | Humble |
| ROS Domain | `55` |
| RMW | `rmw_fastrtps_cpp` |
| DDS transport | `UDPv4` |
| Ubuntu package | `robot_lower_gateway` |
| Ubuntu node | `/ubuntu_lower_gateway` |
| 上位机控制 node | `/robot_lower_gateway` |

> `/ubuntu_lower_gateway` 和 `/robot_lower_gateway` 是两个不同节点。前者在 Ubuntu 下位机
> 运行，后者在上位机运行，不能改成同名。

## 1. 功能边界和数据流

`robot_lower_gateway` 是目标机器人原控制工作空间旁边的适配层，不是 EtherCAT 驱动，也
不替代 `ros2_control`、controller、硬件接口或 workspace supervisor。

默认只读数据流：

```text
机器人原控制栈
  joint_states / power / mode / motion / TCP / execution / workspace
                              |
                              v
                  robot_lower_gateway Backend
               关节名称、状态码和新鲜度归一化
                              |
                              v
       /ubuntu_lower_gateway/joint_states
       /ubuntu_lower_gateway/diagnostics
                              |
                         ROS 2 / DDS
                              |
                              v
                        上位机 gateway
```

完成仿真验收后可以显式打开命令代理：

```text
上位机
  -> /ubuntu_lower_gateway/<command>
  -> 目标工作空间已有 native service
  -> 原 controller / hardware / EtherCAT
```

代理只调用已有 ROS service，不创建第二个硬件对象、不生成 CiA402 controlword、不直接写
PDO。原控制服务和 EtherCAT master 必须始终只有一个 owner。

默认输出：

| 名称 | 类型 | 说明 |
|---|---|---|
| `/ubuntu_lower_gateway/joint_states` | `sensor_msgs/msg/JointState` | 固定 14 轴顺序的有效反馈 |
| `/ubuntu_lower_gateway/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 输入完整性、新鲜度和驱动状态 |

可选代理服务：

| 网关入口 | 转发到当前原生服务 |
|---|---|
| `/ubuntu_lower_gateway/set_robot_power` | `/set_robot_power` |
| `/ubuntu_lower_gateway/set_control_mode` | `/set_arm_control_mode` |
| `/ubuntu_lower_gateway/joint_batch_control` | `/arm/joint_batch_control` |
| `/ubuntu_lower_gateway/joint_absolute_control` | `/arm_absolute_control` |
| `/ubuntu_lower_gateway/cartesian_increment_control` | `/cartesian_increment_control` |
| `/ubuntu_lower_gateway/cartesian_absolute_control` | `/cartesian_absolute_control` |

Workspace 生命周期不经过上述代理，仍使用：

```text
/workspace/control
/workspace/status
```

## 2. 部署前安全条件

1. 首次部署和网络联调保持电机失能，不发送关节或笛卡尔运动。
2. 物理急停必须独立可用，ROS 命令不能作为唯一急停手段。
3. 先用只读模式验收，再在 SIM 中验收命令代理，最后才允许 REAL 分阶段测试。
4. 不允许下位机和上位机出现重复 gateway 节点。
5. 不允许旧 `robot.service` 与受管 SIM/REAL stack 同时运行。
6. REAL stack 不得开机自动启动，电机也不得自动使能。

## 3. 网络拓扑和静态 IP

### 3.1 先识别真实网口

在 Ubuntu 下位机执行：

```bash
hostname
ip -br link
ip -br addr
ip route
nmcli device status
nmcli connection show
```

用网线拔插、MAC、交换机端口和 `ethtool` 共同确认控制网口，不要根据历史接口名猜测：

```bash
ethtool enp8s0
cat /sys/class/net/enp8s0/address
cat /sys/class/net/enp8s0/operstate
cat /sys/class/net/enp8s0/carrier
```

当前机器的 `enp3s0`、`enp4s0` 是 EtherCAT 专用网口，不配置普通 IP、不设默认路由、
不桥接到控制网络。迁移时也必须把控制网和 EtherCAT 网分开。

### 3.2 配置 Ubuntu 下位机地址

下面命令仅适用于已确认控制网口为 `enp8s0` 的当前机器：

```bash
if nmcli connection show robot-network-lower >/dev/null 2>&1; then
  sudo nmcli connection modify robot-network-lower \
    connection.interface-name enp8s0 \
    ipv4.method manual \
    ipv4.addresses 192.168.2.20/24 \
    ipv4.never-default yes \
    ipv6.method disabled
else
  sudo nmcli connection add \
    type ethernet \
    ifname enp8s0 \
    con-name robot-network-lower \
    ipv4.method manual \
    ipv4.addresses 192.168.2.20/24 \
    ipv4.never-default yes \
    ipv6.method disabled
fi

sudo nmcli connection up robot-network-lower
```

`ipv4.never-default yes` 可避免机器人专网覆盖设备原来的互联网默认路由。

验证：

```bash
ip -4 addr show dev enp8s0
ip route show
ping -c 4 192.168.2.10
ping -M do -s 1472 -c 10 192.168.2.10
```

预期控制网段路由为：

```text
192.168.2.0/24 dev enp8s0
```

### 3.3 配置上位机地址

在新上位机上先查真实接口名。以下假设为 `eno1`：

```bash
ip -br link
nmcli device status

if nmcli connection show robot-network-upper >/dev/null 2>&1; then
  sudo nmcli connection modify robot-network-upper \
    connection.interface-name eno1 \
    ipv4.method manual \
    ipv4.addresses 192.168.2.10/24 \
    ipv4.never-default yes \
    ipv6.method disabled
else
  sudo nmcli connection add \
    type ethernet \
    ifname eno1 \
    con-name robot-network-upper \
    ipv4.method manual \
    ipv4.addresses 192.168.2.10/24 \
    ipv4.never-default yes \
    ipv6.method disabled
fi

sudo nmcli connection up robot-network-upper
ping -c 4 192.168.2.20
```

同一网络内不得同时保留旧上位机和新上位机的 `192.168.2.10`。切换电脑前先关闭旧机控制
网口或断开其网线，避免 IP 冲突和重复命令源。

### 3.4 防火墙和组播

ROS 2 自动发现依赖 UDP 组播。确认接口包含 `MULTICAST`：

```bash
ip link show enp8s0
sudo ip link set dev enp8s0 multicast on
```

如果 UFW 已启用，只放行机器人专网，不建议直接关闭整机防火墙：

```bash
sudo ufw status
sudo ufw allow from 192.168.2.0/24
sudo ufw allow to 192.168.2.0/24
```

跨 VLAN、经过不转发组播的交换网络或容器网络时，自动发现可能失败。此时需要统一配置
Fast DDS Discovery Server 或静态 peers；不能只在一端设置 `ROS_DISCOVERY_SERVER`。

## 4. ROS 2 和 DDS 环境

两端必须使用相同 Domain、兼容 RMW 和相同 `robot_control_msg` 接口。推荐在每台机器建立
环境脚本，终端和 systemd 都使用同一组值：

```bash
source /opt/ros/humble/setup.bash
source /对应工作空间/install/setup.bash

export ROS_DOMAIN_ID=55
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

unset ROS_DISCOVERY_SERVER
unset CYCLONEDDS_URI
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset FASTDDS_DEFAULT_PROFILES_FILE
```

不要在一端使用 Cyclone DDS，另一端又残留 Fast DDS profile。修改环境后建议关闭旧 ROS
daemon，再使用不依赖 daemon 的发现命令检查：

```bash
ros2 daemon stop || true
ros2 node list --no-daemon --spin-time 5
```

上下位机时间也应同步：

```bash
timedatectl
date --iso-8601=ns
```

### 4.1 消息接口一致性

两端分别检查：

```bash
ros2 interface show robot_control_msg/msg/WorkspaceControl
ros2 interface show robot_control_msg/msg/WorkspaceStatus
ros2 interface show robot_control_msg/msg/ArmPowerStatus
ros2 interface show robot_control_msg/srv/SetRobotPower
ros2 interface show robot_control_msg/srv/JointBatchControl
```

发布版本时保存接口文件校验值：

```bash
find src/robot_control_msg/msg src/robot_control_msg/srv \
  -type f -print0 | sort -z | xargs -0 sha256sum > robot_control_msg.sha256
```

在另一端对同一批接口文件执行同样命令并比较。不能在 gateway 包中重新定义字段不同的
同名接口。

## 5. 安装依赖

推荐 Ubuntu 22.04 + ROS 2 Humble。下位机安装基础依赖：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  python3-colcon-common-extensions python3-rosdep \
  ros-humble-rmw-fastrtps-cpp \
  ros-humble-diagnostic-msgs \
  ros-humble-sensor-msgs \
  ros-humble-pluginlib
```

首次使用 `rosdep`：

```bash
sudo rosdep init
rosdep update
```

如果 `rosdep init` 提示已经存在，可以直接继续。

## 6. 当前工作空间中的集成部署

当前代码位置：

```text
/home/user/joint_controller/src/robot_lower_gateway
```

构建：

```bash
cd /home/user/joint_controller
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install \
  --packages-select robot_control_msg robot_lower_gateway robot_control \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

检查安装产物：

```bash
ros2 pkg prefix robot_lower_gateway
ros2 pkg executables robot_lower_gateway
test -x install/robot_lower_gateway/lib/robot_lower_gateway/run_gateway.sh
```

## 7. 独立 overlay 部署（推荐迁移方式）

为了不修改目标机器人原工作空间，正式迁移建议单独建立：

```text
/opt/robot_lower_gateway_ws
```

假设目标原工作空间为 `/home/robot/target_robot_ws`，运行用户为 `robot`：

```bash
sudo mkdir -p /opt/robot_lower_gateway_ws/src
sudo chown -R robot:robot /opt/robot_lower_gateway_ws

cp -a /交付目录/robot_lower_gateway \
  /opt/robot_lower_gateway_ws/src/

cd /opt/robot_lower_gateway_ws
source /opt/ros/humble/setup.bash
source /home/robot/target_robot_ws/install/setup.bash

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select robot_lower_gateway \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

构建时先 source 目标工作空间，是为了找到匹配版本的 `robot_control_msg`。运行时环境顺序
固定为：

```text
/opt/ros/humble/setup.bash
目标机器人工作空间 install/setup.bash
robot_lower_gateway_ws install/setup.bash
```

也可以把包直接复制到目标工作空间的 `src/`，但会共享 `build/install/log`，不利于独立
升级和回滚。

## 8. 配置机器人 Profile

当前模板：

```text
src/robot_lower_gateway/config/erobot_v1.yaml
```

运行时建议使用独立配置：

```bash
sudo install -d -m 0755 /etc/robot-lower-gateway
sudo install -m 0644 \
  /home/user/joint_controller/install/robot_lower_gateway/share/robot_lower_gateway/config/erobot_v1.yaml \
  /etc/robot-lower-gateway/robot.yaml
```

配置根节点必须保持：

```yaml
ubuntu_lower_gateway:
  ros__parameters:
```

关键配置：

```yaml
backend_plugin: robot_lower_gateway/ErobotBackend
publish_rate_hz: 20.0

enable_command_proxy: false
command_proxy_prefix: "~/"

canonical_joint_names: [ljoint1, ljoint2, ljoint3, ljoint4, ljoint5, ljoint6, ljoint7,
                        rjoint1, rjoint2, rjoint3, rjoint4, rjoint5, rjoint6, rjoint7]
native_joint_names:    [ljoint1, ljoint2, ljoint3, ljoint4, ljoint5, ljoint6, ljoint7,
                        rjoint1, rjoint2, rjoint3, rjoint4, rjoint5, rjoint6, rjoint7]
```

两个关节数组必须都是 14 项、没有重复，并按下标一一对应。`canonical_joint_names` 是给
上层使用的固定名称，`native_joint_names` 是目标控制栈的真实名称。

如果目标机器人只是 topic 名变化，修改 `topics.*`。如果电机状态码变化，必须根据真实
驱动反馈修改：

```yaml
drive_status:
  enabled_codes: [39]
  unavailable_codes: [0]
```

`39` 和 `0` 只适用于当前 erobot。新电机的 UNKNOWN、未连接和 ENABLED 状态必须先实测，
不能照抄，也不能把未知码列入 `enabled_codes` 让检查通过。

超时值要大于实际发布周期并留有抖动余量，不要用超长 timeout 隐藏丢包：

```yaml
stale_timeout_ms:
  joint_state: 250
  power_status: 500
  control_mode: 500
  motion_status: 500
  tcp_pose: 500
  execution: 3000
```

## 9. 前台只读启动和验收

先保持命令代理关闭。目标控制栈未启动时，网关 diagnostics 为 waiting 是正常的。

```bash
cd /home/user/joint_controller
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=55
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset ROS_DISCOVERY_SERVER CYCLONEDDS_URI
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE

ros2 launch robot_lower_gateway lower_gateway.launch.py \
  config_file:=/etc/robot-lower-gateway/robot.yaml
```

另开终端检查：

```bash
ros2 node list --no-daemon --spin-time 5 | sort
ros2 node info /ubuntu_lower_gateway

ros2 topic echo /ubuntu_lower_gateway/joint_states \
  sensor_msgs/msg/JointState --once

ros2 topic echo /ubuntu_lower_gateway/diagnostics \
  diagnostic_msgs/msg/DiagnosticArray --once

ros2 topic hz /ubuntu_lower_gateway/joint_states
```

验收要求：

- `/ubuntu_lower_gateway` 只有一个实例；
- 关节反馈为固定 14 轴、有限值、左右臂没有交换；
- diagnostics 显示输入完整且新鲜；
- 默认没有 `/ubuntu_lower_gateway/*` 控制 service；
- 节点没有 command publisher、EtherCAT owner 或硬件对象；
- 停止原反馈后 stale 保护测试只在 SIM 中执行。

## 10. 当前工作空间的 systemd 自启动

当前仓库已经提供集成安装脚本：

```bash
cd /home/user/joint_controller
sudo ./scripts/install_workspace_control_services.sh
```

它会安装并启用：

```text
robot-lower-gateway.service
joint-controller-supervisor.service
```

同时明确禁用以下开机启动：

```text
joint-controller-stack-sim.service
joint-controller-stack-real.service
joint-controller-rviz.service
```

因此重启后只有通信 gateway 和 workspace supervisor 监听命令，SIM/REAL 不会自动运行，
EtherCAT 和电机不会自动上电。安装脚本还会停用可能产生重复控制栈的旧
`robot.service`。

运行配置：

```text
/etc/robot-lower-gateway/robot.yaml
/etc/robot-lower-gateway/gateway.env
```

首次安装生成的 `gateway.env` 应为：

```ini
GATEWAY_OVERLAY_SETUP=/home/user/joint_controller/install/setup.bash
ROBOT_GATEWAY_CONFIG=/etc/robot-lower-gateway/robot.yaml
ROS_DOMAIN_ID=55
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ROBOT_GATEWAY_ENABLE_COMMAND_PROXY=false
ROBOT_GATEWAY_COMMAND_PROXY_PREFIX=~/
```

安装脚本会保留已有 `gateway.env`，更新代码后不会自动覆盖已审核的 proxy 设置。

检查自启动策略：

```bash
systemctl is-enabled robot-lower-gateway.service
systemctl is-active robot-lower-gateway.service
systemctl is-enabled joint-controller-supervisor.service
systemctl is-active joint-controller-supervisor.service

systemctl is-enabled joint-controller-stack-sim.service || true
systemctl is-enabled joint-controller-stack-real.service || true

sudo systemctl status robot-lower-gateway.service --no-pager -l
sudo journalctl -u robot-lower-gateway.service -n 100 --no-pager
```

预期 gateway、supervisor 为 `enabled/active`，SIM 和 REAL 为 `disabled/inactive`。

### 10.1 重启验收

确认电机失能后再安排重启：

```bash
sudo reboot
```

系统恢复后执行：

```bash
source /opt/ros/humble/setup.bash
source /home/user/joint_controller/install/setup.bash
export ROS_DOMAIN_ID=55 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

ros2 node list --no-daemon --spin-time 5
ros2 topic echo /workspace/status \
  robot_control_msg/msg/WorkspaceStatus \
  --qos-reliability reliable \
  --qos-durability transient_local --once
```

预期只有 `/ubuntu_lower_gateway`、`/workspace_supervisor_ubuntu` 等监听节点，workspace 为
`STOPPED/UNKNOWN`，没有自动 power on。

## 11. 独立 overlay 的 systemd 安装

独立工作空间不使用当前固定路径的集成安装脚本，应使用包内模板。

创建环境文件：

```ini
# /etc/robot-lower-gateway/gateway.env
ROBOT_UNDERLAY_SETUP=/home/robot/target_robot_ws/install/setup.bash
GATEWAY_OVERLAY_SETUP=/opt/robot_lower_gateway_ws/install/setup.bash
ROBOT_GATEWAY_CONFIG=/etc/robot-lower-gateway/robot.yaml
ROS_DOMAIN_ID=55
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ROBOT_GATEWAY_ENABLE_COMMAND_PROXY=false
ROBOT_GATEWAY_COMMAND_PROXY_PREFIX=~/
```

从模板生成 unit，以下运行用户为 `robot`：

```bash
sed \
  -e 's|@ROBOT_USER@|robot|g' \
  -e 's|@ROBOT_GROUP@|robot|g' \
  -e 's|@GATEWAY_WORKSPACE@|/opt/robot_lower_gateway_ws|g' \
  -e 's|@GATEWAY_RUNNER@|/opt/robot_lower_gateway_ws/install/robot_lower_gateway/lib/robot_lower_gateway/run_gateway.sh|g' \
  /opt/robot_lower_gateway_ws/src/robot_lower_gateway/systemd/robot-lower-gateway.service.in \
  > /tmp/robot-lower-gateway.service

grep -n '@' /tmp/robot-lower-gateway.service
sudo install -m 0644 /tmp/robot-lower-gateway.service \
  /etc/systemd/system/robot-lower-gateway.service

sudo systemctl daemon-reload
sudo systemctl enable --now robot-lower-gateway.service
```

`grep` 必须没有输出。检查日志中使用的是正确 underlay、overlay 和 YAML。停止 gateway
不会自动停止原控制栈，也不会自动失能电机。

## 12. 分阶段启用命令代理

只有只读验收和 SIM A/B 测试通过后，才把：

```ini
ROBOT_GATEWAY_ENABLE_COMMAND_PROXY=true
```

写入 `/etc/robot-lower-gateway/gateway.env`，然后：

```bash
sudo systemctl restart robot-lower-gateway.service
ros2 run robot_lower_gateway verify_gateway_proxy.sh
```

`verify_gateway_proxy.sh` 只检查六个服务的名称和类型，不发送请求。

确认拓扑：

```bash
ros2 node info /ubuntu_lower_gateway
ros2 service list -t | grep ubuntu_lower_gateway
ros2 service type /set_robot_power
ros2 service type /ubuntu_lower_gateway/set_robot_power
```

原生 `/set_robot_power` 只能有原 controller 的一个 server；命名空间代理只能由
`/ubuntu_lower_gateway` 提供。通过 `ros2 node list` 找到原 controller 节点后，使用
`ros2 node info <原控制节点>` 确认它拥有原生 service。禁止把
`command_proxy_prefix` 改为 `/`。

SIM 中先做无运动的失能请求：

```bash
ros2 service call /ubuntu_lower_gateway/set_robot_power \
  robot_control_msg/srv/SetRobotPower "{enable: false}"
```

再逐项做原生入口与代理入口 A/B 对比。请求字段、最终响应、超时和失败原因必须一致；原生
service 不存在时代理必须失败，不能返回伪成功。REAL 首次验收仍按“失能、模式、反馈、
单次使能、小幅运动”的现场安全流程执行，不能一键连续测试。

## 13. 更换或迁移上位机电脑

`robot_lower_gateway` 继续部署在 Ubuntu 下位机。新上位机需要部署的是上位机 gateway/CLI
工作空间和同版本 `robot_control_msg`，不是把 Ubuntu 的 EtherCAT 或 hardware package
复制过去。

### 13.1 新上位机准备

1. 安装与现系统兼容的 Ubuntu、ROS 2 Humble 和 `rmw_fastrtps_cpp`。
2. 将控制网口设为 `192.168.2.10/24`，确认旧上位机已断开。
3. 部署上位机独立工作空间，例如 `/home/<user>/robot_gateway_ws`。
4. 编译并 source 上位机 workspace。
5. 设置与下位机完全一致的 Domain/RMW/DDS 环境。
6. 校验 `robot_control_msg` 接口哈希。
7. 先只做节点、topic 和 service 发现，不启动 SIM/REAL，不上电。

交付时复制源码和配置，不复制另一台机器生成的 `build/`、`install/`、`log/`：

```text
robot_gateway_ws/src/
├── robot_control_msg
├── robot_lower_gateway
├── robot_gateway_client
├── robotctl_cli
└── robot_gateway_bringup
```

在新电脑重新构建：

```bash
mkdir -p /home/<user>/robot_gateway_ws/src
cp -a /交付目录/robot_gateway_ws/src/. \
  /home/<user>/robot_gateway_ws/src/

cd /home/<user>/robot_gateway_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ros2 pkg executables robotctl_cli
ros2 pkg prefix robot_gateway_bringup
```

如果上位机项目提供自己的 systemd 安装脚本，应从新工作空间重新安装，不要复制旧机器
`/etc/systemd` 或 `~/.config/systemd` 中带绝对路径的 unit。安装后检查：

```bash
systemctl --user list-unit-files | grep -E 'robot-gateway|jetson-planner'
systemctl --user list-units --all | grep -E 'robot-gateway|jetson-planner'
```

旧 `jetson-planner@.service` 不得与新 `robot-gateway@sim/real` 同时运行。SIM/REAL profile
也不应在未选择模式时自动启动。

CLI 必须来自新工作空间：

```bash
type -a robotctl
readlink -f "$(command -v robotctl)"
```

若 PATH 中存在旧 `~/.local/bin/robotctl`，应按上位机项目的安装方式更新该入口，不能继续
指向旧 `/home/<user>/joint_controller` 或 `jetson_planner`。

发现检查：

```bash
ping -c 4 192.168.2.20
ros2 node list --no-daemon --spin-time 5
ros2 topic info /workspace/status --verbose
ros2 topic info /ubuntu_lower_gateway/diagnostics --verbose
ros2 service type /ubuntu_lower_gateway/set_robot_power
```

如果下位机代理仍为只读，最后一条服务不存在是正常的；先完成只读反馈联调。

### 13.2 上位机需要遵守的接口契约

- START/STOP 使用 `/workspace/control` 和 `/workspace/status`；
- 反馈优先读取 `/ubuntu_lower_gateway/joint_states` 和 diagnostics；
- proxy 启用后，命令指向 `/ubuntu_lower_gateway/*`；
- 每个命令必须等待下位机最终响应和反馈交叉确认；
- `command_enabled` 只表示命令请求，不能当作实际 14 轴 enabled；
- status `0`、UNKNOWN、stale 或不完整反馈必须拒绝 REAL 动作；
- 禁止上位机绕过 proxy 直接访问 EtherCAT/PDO。

### 13.3 WorkspaceControl source 白名单

当前 Ubuntu supervisor 配置为：

```text
WORKSPACE_CONTROL_ALLOWED_SOURCE=jetson_192_168_2_10
```

这是 `WorkspaceControl.source` 字段的逻辑标识，不是操作系统根据源 IP 自动完成的网络 ACL。
新上位机 gateway 发送的 `source` 必须与它完全一致。

若新电脑继续使用 `192.168.2.10`，可保留此值。若正式修改上位机 IP，建议同时修改：

1. 新上位机静态 IP；
2. Ubuntu 网络连通配置；
3. 上位机发送的 `WorkspaceControl.source`；
4. Ubuntu `joint-controller-supervisor.service` 中的
   `WORKSPACE_CONTROL_ALLOWED_SOURCE`；
5. 相关防火墙规则和部署文档。

修改 unit 后：

```bash
sudo systemctl daemon-reload
sudo systemctl restart joint-controller-supervisor.service
```

不要只改 IP 字符串却保留不匹配的逻辑 source，也不要把 `allowed_source` 设为空来绕过
命令源校验。

### 13.4 新上位机联调顺序

1. 断开旧上位机，确认无 IP 冲突和重复 `/robot_lower_gateway`。
2. 新上位机发现 `/ubuntu_lower_gateway` 与 `/workspace_supervisor_ubuntu`。
3. 读取 `/workspace/status`，要求 `STOPPED/UNKNOWN/accepted=true`。
4. 读取 diagnostics，确认网络与接口版本正确。
5. 启动 SIM，完成状态、模式、power off 和停止测试。
6. SIM 中完成 proxy A/B 测试。
7. 现场安全条件满足后才进行 REAL 只读 readiness。
8. REAL 检测到真实驱动且仍为 `0/14` 后，才进入后续单次使能测试。

## 14. 迁移到不同机器人或不同电机

### 14.1 外观和 14 轴结构相同，仅名称或状态码不同

新增一份 YAML profile，不修改网关核心：

- 映射 `native_joint_names` 到固定 `canonical_joint_names`；
- 修改原生 topic/service 名；
- 填写新电机经过验证的 enabled/unavailable 状态码；
- 按真实反馈周期设置 stale timeout。

### 14.2 原生 ROS 消息类型不同

在包内新增 Backend plugin：

```text
include/robot_lower_gateway/<new_backend>.hpp
src/<new_backend>.cpp
plugins.xml
```

新 Backend 订阅目标工作空间正式反馈并生成统一 snapshot。更新 `CMakeLists.txt`、
`plugins.xml` 和 profile 的 `backend_plugin`，但不在 gateway 主节点中堆积机器人型号判断。

### 14.3 命令接口不同

优先要求目标工作空间提供正式 ROS service/action，然后让 proxy 调用它。若目标控制器使用
`ros2_control` command interface，应通过正式 controller API 接入，不能由 gateway new
另一个 hardware interface。

若只有裸 EtherCAT PDO 或厂商写 API，必须在目标硬件驱动/controller 内实现唯一写入链路。
这已经不是只改 YAML 的迁移，不能让 gateway 和原驱动同时占用总线。

### 14.4 每台新机器必须重新验证

- 14 轴名称、方向、零位和单位；
- 未连接、失能、使能和故障状态码；
- 控制模式枚举；
- TCP 坐标系、四元数约定和单位；
- 原生 service 最终返回语义；
- timeout、并发互斥和 stale 保护；
- 原硬件/controller/EtherCAT owner 唯一性。

## 15. 日常启动和停止

开机后通信 gateway 和 supervisor 应自动运行，工作空间默认停止。上位机标准流程：

```bash
robotctl down
robotctl up sim
robotctl start sim
robotctl status
```

REAL 必须显式确认：

```bash
robotctl down
robotctl up real --confirm-real
robotctl start real --confirm-real
robotctl status
```

正常停止顺序：

```bash
robotctl power off
robotctl stop
robotctl down
```

必须先失能再停止 workspace；workspace 停止后原电源服务可能已经退出，此时再 power off 会
超时。`robotctl down` 只停止上位机 gateway，不等于下位机失能或停止。

## 16. 故障排查

### 16.1 能 ping，但看不到 ROS 节点

```bash
printenv | grep -E 'ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION|FASTDDS'
ros2 node list --no-daemon --spin-time 5
sudo ufw status
ip link show enp8s0
```

重点检查 Domain、RMW、组播、防火墙、残留 discovery/profile 环境变量以及两端是否 source
了正确 workspace。

### 16.2 diagnostics 一直 WAITING 或 stale

- 原控制栈未启动；
- YAML topic 名或消息类型错误；
- QoS 不兼容；
- 关节消息缺少某一轴、名称重复或含 NaN/Inf；
- timeout 小于实际发布抖动；
- 下位机运行了旧 install 产物。

检查：

```bash
ros2 topic list -t
ros2 topic info /arm/joint_states --verbose
ros2 topic echo /arm/joint_states --once
ros2 topic echo /ubuntu_lower_gateway/diagnostics --once
```

### 16.3 服务可见但调用超时

```bash
ros2 service type /ubuntu_lower_gateway/set_robot_power
ros2 service type /set_robot_power
ros2 node info /ubuntu_lower_gateway
sudo journalctl -u robot-lower-gateway.service -n 150 --no-pager
```

确认 proxy 和 native service 各有一个正确 owner。代理等待的是原生服务最终响应，不能通过
缩短调用链或伪造成功解决超时。

### 16.4 systemd 使用了旧路径

```bash
systemctl cat robot-lower-gateway.service
sudo cat /etc/robot-lower-gateway/gateway.env
ros2 pkg prefix robot_lower_gateway
readlink -f /proc/$(pgrep -n robot_lower_gateway_node)/exe
```

修改代码后必须重新编译并重启服务；已运行进程不会自动加载新二进制。

### 16.5 重复节点或重复服务

```bash
ros2 node list --no-daemon --spin-time 5 | sort
pgrep -af 'robot_lower_gateway|workspace_supervisor|ros2 launch'
systemctl list-units --type=service | grep -E 'robot|joint-controller'
```

同名节点、原生 service 或旧控制栈重复时先停止并清理部署入口，不要继续 REAL 测试。

## 17. 更新和回滚

在电机失能、workspace 停止后更新：

```bash
sudo systemctl stop robot-lower-gateway.service

cd /home/user/joint_controller
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_lower_gateway

sudo systemctl start robot-lower-gateway.service
sudo journalctl -u robot-lower-gateway.service -n 100 --no-pager
```

独立 overlay 则先 source 目标 underlay，再构建 gateway overlay。回滚必须同时恢复已验证的
源码、`robot.yaml` 和接口版本，不能只替换 `.so`。

## 18. 最终验收清单

- [ ] 上位机和下位机静态 IP 唯一且互相可达。
- [ ] 控制网口与 EtherCAT 专用网口没有混用。
- [ ] 两端时间、ROS Domain、RMW 和 DDS transport 一致。
- [ ] `robot_control_msg` 接口版本或校验值一致。
- [ ] `/ubuntu_lower_gateway` 和 `/robot_lower_gateway` 各只有一个实例。
- [ ] 14 轴 canonical/native 映射逐项确认。
- [ ] 规范化关节反馈完整、有限、顺序固定。
- [ ] diagnostics 能识别 waiting、stale、unavailable 和 enabled。
- [ ] 默认只读模式没有命令 service、command publisher 或 EtherCAT owner。
- [ ] gateway 和 supervisor 开机自启，SIM/REAL 均不自启。
- [ ] 重启后 workspace 为 `STOPPED/UNKNOWN`，电机没有自动使能。
- [ ] proxy 只在完成 SIM A/B 测试后启用。
- [ ] proxy 与原生 service 的 owner、响应、失败和超时语义正确。
- [ ] 新上位机接入前旧上位机已断开，不存在 IP 或节点冲突。
- [ ] 新电机状态码和 Backend 根据真实接口验证，没有照抄旧状态语义。
- [ ] REAL 测试遵循现场急停、单次上电和逐步运动验收流程。
