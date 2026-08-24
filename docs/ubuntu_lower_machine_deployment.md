# Ubuntu 下位机部署、自启动与验收

本文适用于当前双臂控制系统的 Ubuntu/x86 下位机。文档基于以下固定部署：

| 项目 | 值 |
| --- | --- |
| Jetson 上位机控制网 IP | `192.168.2.10/24` |
| Ubuntu 下位机控制网 IP | `192.168.2.20/24` |
| Ubuntu 用户 | `user` |
| Ubuntu 工作空间 | `/home/user/joint_controller` |
| ROS 2 | Humble |
| ROS Domain | `55` |
| RMW | `rmw_fastrtps_cpp` |
| DDS transport | `UDPv4` |

如果用户名、工作空间路径或 IP 不同，必须同步修改 `scripts/`、`systemd/` 和
supervisor 的 source 白名单。当前文件不是可任意换目录运行的通用安装包。

## 1. 上位机文档需要补充和修正的内容

现有 Jetson 文档的主体流程可用，但正式发布前建议修正以下内容：

1. 统一使用新工作空间 `/home/yuling/robot_gateway_ws`。旧路径
   `/home/yuling/joint_controller` 仅用于历史版本，不应混在新部署步骤中。
2. 将 `souce install/setup.bash` 修正为 `source install/setup.bash`。
3. `robotctl up sim/real` 只管理 Jetson gateway/planner，不启动 Ubuntu workspace。
4. `robotctl down` 也只停止 Jetson gateway/planner，不能代替 `robotctl stop`。
5. 真机顺序必须是 `up real -> start real -> status -> power off -> mode position ->`
   `检查关节反馈 -> power on`，不能在 workspace 为 `STOPPED` 时执行 mode 或 power。
6. 停止顺序统一为 `power off -> stop -> down`。workspace 停止后再执行 power off
   会因为 Ubuntu 电源服务已退出而超时。
7. `robotctl ready sim` 只用于仿真。真机不得提供无确认的一键 ready。
8. 增加上下位机时间同步、消息接口版本一致性、重复节点和旧 planner 清理检查。
9. 明确 `robotctl status` 中 transient-local execution 消息可能是最后一次执行结果；
   判断本次动作必须使用 command/generation 和新的 TCP、motion 反馈。
10. 文档中的命令只保留一种入口：安装后优先直接使用 `robotctl`；仅在未安装 CLI
    时说明它等价于 `ros2 run robotctl_cli robotctl`。

## 2. 安全的开机策略

安装完成后只有以下服务开机自启动：

```text
joint-controller-supervisor.service
```

它只监听 `/workspace/control`、发布 `/workspace/status`，不会启动 EtherCAT、不会使能
电机，也不会发送运动命令。

以下服务只安装、不 enable，必须由 Jetson 明确选择后启动：

```text
joint-controller-stack-sim.service
joint-controller-stack-real.service
joint-controller-rviz.service
```

禁止把 REAL stack 或旧的 `robot.service` 设置为开机自启动。物理急停必须独立可用，
不能把 ROS 命令当作唯一急停手段。

## 3. 下位机基础环境

### 3.1 系统与时间

推荐 Ubuntu 22.04 + ROS 2 Humble。首先确认：

```bash
lsb_release -a
uname -r
timedatectl
```

上下位机时间应同步。可使用现有局域网 NTP/chrony；至少确认两台机器时间差不会导致
日志无法关联：

```bash
date --iso-8601=ns
timedatectl timesync-status
```

### 3.2 控制网口

先区分三个类别的网口：

- 一个 Jetson/Ubuntu ROS 控制网口；
- EtherCAT Master0 专用网口；
- EtherCAT Master1 专用网口。

不要把 `192.168.2.20` 配置到 EtherCAT 专用口。

查看接口和 NetworkManager 连接：

```bash
ip -br link
ip -br addr
nmcli device status
nmcli connection show
```

以下假设 Ubuntu 控制网口为 `eno1`，现场必须替换为真实名称：

```bash
sudo nmcli connection add \
  type ethernet \
  ifname eno1 \
  con-name robot-network-lower \
  ipv4.method manual \
  ipv4.addresses 192.168.2.20/24 \
  ipv4.never-default yes \
  ipv6.method disabled

sudo nmcli connection up robot-network-lower
```

验证：

```bash
ip -4 addr show dev eno1
ip route
ping -c 4 192.168.2.10
ping -M do -s 1472 -c 10 192.168.2.10
ethtool eno1
```

应满足：

```text
192.168.2.20/24
192.168.2.0/24 dev eno1
Link detected: yes
MTU 1500
```

接口标志应包含 `MULTICAST`。必要时启用：

```bash
sudo ip link set dev eno1 multicast on
```

若 UFW 已启用，允许专用机器人子网，不要直接关闭整机防火墙：

```bash
sudo ufw status
sudo ufw allow from 192.168.2.0/24
sudo ufw allow to 192.168.2.0/24
```

## 4. ROS 2 与构建依赖

安装 ROS 2 Humble 后，准备基础工具：

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-rviz2 \
  ros-humble-rmw-fastrtps-cpp
```

首次使用 rosdep 时：

```bash
sudo rosdep init
rosdep update
```

如果系统已经初始化，`rosdep init` 提示文件存在可以忽略。安装工作空间声明的依赖：

```bash
cd /home/user/joint_controller
source /opt/ros/humble/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y
```

本项目还依赖 Pinocchio。Ruckig 和 qpOASES 已包含在仓库中；Pinocchio 必须使用项目
验证过的版本。检查动态库：

```bash
ldconfig -p | grep -E 'pinocchio|eigenpy'
```

如果 rosdep 无法解析 `pinocchio`，先按项目固定版本完成安装，再继续编译，不要随意混用
系统包和 `/usr/local` 中的不同版本。

## 5. EtherCAT/IGH 环境

仿真部署可先跳过本节；真机部署必须完成。

### 5.1 保留 EtherCAT 专用网口

记录两个 EtherCAT 网口的接口名和 MAC：

```bash
ip -br link
ethtool <ethercat_master0_if>
ethtool <ethercat_master1_if>
```

EtherCAT 网口不配置普通 IP，不参与默认路由，也不要与控制网口桥接。项目的
`robot_ethercat_host_setup.sh --apply` 会把三个 EtherCAT 接口持久设置为
NetworkManager unmanaged，避免 NetworkManager 在专用口上持续 DHCP。

### 5.2 IGH Master 配置

确认已安装项目验证过的 IgH/EtherLab Master。现场日志使用过 `1.6.9`。检查：

```bash
ethercat version
sudo sed -n '1,160p' /usr/local/etherlab/etc/sysconfig/ethercat
sudo sed -n '1,160p' /usr/local/etherlab/etc/ethercat.conf
```

完整手臂加升降系统必须配置三个 master。下面只是结构示例，MAC 必须以现场
`ip link` 输出为准：

```bash
MASTER0_DEVICE="90:b3:d5:54:30:0e"
MASTER0_DRIVER="generic"
MASTER1_DEVICE="90:b3:d5:54:30:0f"
MASTER1_DRIVER="generic"
MASTER2_DEVICE="90:b3:d5:54:30:10"
MASTER2_DRIVER="generic"
DEVICE_MODULES="generic"
```

不要直接复制示例 MAC 到另一台机器。配置完成后：

```bash
sudo /home/user/joint_controller/install/joint_hardware/lib/joint_hardware/robot_ethercat_host_setup.sh \
  --apply enp3s0 enp4s0 enp5s0
```

IgH 的 init.d 脚本读取 `etc/sysconfig/ethercat`，systemd unit 默认读取
`etc/ethercat.conf`。部署脚本会同步这两份配置；两者不得保留不同的 master
网口映射。脚本会打印 `ethercat master` 和 `ethercat slaves` 的结果，随后停止
EtherCAT 内核主站；它不启动 IGH 用户态驱动、ROS REAL stack 或电机使能。

命令是 `ethercat master`，不是 `ethercat masters` 或 `ethercat mastert`。

未启动应用程序时看到 `Phase: Idle`、`Active: no`、从站为 `PREOP` 可以是正常状态；
启动手臂 REAL stack 后 Master0/1 必须变为应用 active，14 个手臂从站应全部进入
`OP`。升降单独使用 Master2，不得用升降配置脚本覆盖 Master0/1。

### 5.3 IGH 用户态驱动

当前 REAL 启动脚本固定使用：

```text
/home/user/joint_controller/src/erobot_igh_driver/build/igh_driver
```

部署时必须保证该文件存在、可执行，并且与当前 14 轴映射和 PDO 配置一致：

```bash
test -x /home/user/joint_controller/src/erobot_igh_driver/build/igh_driver
ls -l /home/user/joint_controller/src/erobot_igh_driver/build/igh_driver
```

如果驱动源代码放在其它位置，必须修改 `start.sh`、`stop.sh` 后重新安装 systemd unit，
不能只在当前终端设置临时变量。

## 6. 工作空间部署与编译

代码必须部署到固定路径：

```bash
sudo mkdir -p /home/user/joint_controller
sudo chown -R user:user /home/user/joint_controller
```

将代码放入该目录后编译：

```bash
cd /home/user/joint_controller
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

检查关键接口和可执行文件：

```bash
ros2 interface show robot_control_msg/msg/WorkspaceControl
ros2 interface show robot_control_msg/msg/WorkspaceStatus
ros2 interface show robot_control_msg/srv/SetRobotPower
ros2 interface show robot_control_msg/srv/JointAbsoluteControl

test -x install/robot_control/lib/robot_control/workspace_supervisor_ubuntu
test -x install/robot_control/lib/robot_control/cartesian_single_control_srv
```

上下位机的 `robot_control_msg` 必须来自同一接口版本。部署包交付时建议保存校验值：

```bash
find src/robot_control_msg/msg src/robot_control_msg/srv \
  -type f -print0 | sort -z | xargs -0 sha256sum
```

### 6.1 可移植下位机网关（可选）

`robot_lower_gateway` 是一个可以单独复制的 ROS 2 package，用于把不同机器人工作空间的反馈转换为固定 14 轴顺序并检查数据新鲜度。它不是当前原控制栈的替代品。
默认只读；完成仿真 A/B 验收后可显式启用命名空间 command proxy。proxy 只调用原工作空间
已有 service，原 controller/hardware/EtherCAT 仍是唯一 owner。

```text
源目录：/home/user/joint_controller/src/robot_lower_gateway
运行节点：/ubuntu_lower_gateway
规范化关节反馈：/ubuntu_lower_gateway/joint_states
健康诊断：/ubuntu_lower_gateway/diagnostics
```

移植到另一个 Ubuntu 工作空间时：

```bash
cp -a /原工作空间/src/robot_lower_gateway \
  /目标工作空间/src/

cd /目标工作空间
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select robot_lower_gateway
source install/setup.bash
```

目标工作空间必须已经提供与 Jetson 完全一致的 `robot_control_msg`。如果消息包不在源码中，应先安装相同版本的二进制包或将接口仓库作为独立依赖部署，不能在网关包内改名重定义。

当前阶段只读运行：

```bash
ros2 launch robot_lower_gateway lower_gateway.launch.py
```

验证：

```bash
ros2 topic echo /ubuntu_lower_gateway/joint_states \
  sensor_msgs/msg/JointState --once

ros2 topic echo /ubuntu_lower_gateway/diagnostics \
  diagnostic_msgs/msg/DiagnosticArray --once

ros2 node info /ubuntu_lower_gateway
```

默认 `/ubuntu_lower_gateway` 不提供 `/set_robot_power`、模式、关节或笛卡尔控制服务。现有
控制服务仍由 `robot_control` 和 `erobot_controller` 提供。

仿真中显式启用 proxy：

```bash
ros2 launch robot_lower_gateway lower_gateway.launch.py \
  enable_command_proxy:=true
```

此时只新增以下命名空间服务，不占用全局 owner：

```text
/ubuntu_lower_gateway/set_robot_power
/ubuntu_lower_gateway/set_control_mode
/ubuntu_lower_gateway/joint_batch_control
/ubuntu_lower_gateway/joint_absolute_control
/ubuntu_lower_gateway/cartesian_increment_control
/ubuntu_lower_gateway/cartesian_absolute_control
```

先用 `SetRobotPower(enable=false)` 做无运动验证，再按原控制栈安全流程逐项测试模式、关节
和笛卡尔代理。响应必须由原生 service 返回并保持字段一致；原生 service 不可用或超时必须
返回失败。未完成仿真 A/B 回归前，不要打开 REAL proxy，也不要把 Jetson 指向新接口。

## 7. 交互终端环境

每个新终端可直接 source 仓库脚本：

```bash
cd /home/user/joint_controller
source scripts/source_lower_machine_env.sh
```

它等价于：

```bash
source /opt/ros/humble/setup.bash
source /home/user/joint_controller/install/setup.bash

export ROS_DOMAIN_ID=55
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

unset ROS_DISCOVERY_SERVER
unset CYCLONEDDS_URI
unset FASTRTPS_DEFAULT_PROFILES_FILE
```

不要把不同的 `ROS_DOMAIN_ID`、CycloneDDS 配置或 discovery server 写入同一用户的
`.bashrc`。systemd 服务不依赖当前终端是否 source，它们使用 unit 和启动脚本中的环境。

## 8. 安装 systemd 与开机监听

安装前确认当前没有 SIM/REAL 栈运行，机械臂已失能：

```bash
systemctl is-active joint-controller-stack-sim.service || true
systemctl is-active joint-controller-stack-real.service || true
```

执行安装：

```bash
cd /home/user/joint_controller
sudo ./scripts/install_workspace_control_services.sh
```

脚本会：

1. 停用旧的 `robot.service`；
2. 安装 supervisor、SIM、REAL 和 RViz unit；
3. 禁止 SIM/REAL/RViz unit 开机自启；
4. enable 并重启 supervisor；
5. 执行 `systemctl daemon-reload`。

验证启动策略：

```bash
systemctl is-enabled joint-controller-supervisor.service
systemctl is-enabled joint-controller-stack-sim.service || true
systemctl is-enabled joint-controller-stack-real.service || true
systemctl is-enabled joint-controller-rviz.service || true

sudo systemctl status joint-controller-supervisor.service --no-pager
```

预期：

```text
joint-controller-supervisor.service: enabled, active (running)
joint-controller-stack-sim.service: disabled
joint-controller-stack-real.service: disabled
joint-controller-rviz.service: disabled
```

重启验收：

```bash
sudo reboot
```

系统回来后，下位机执行：

```bash
cd /home/user/joint_controller
source scripts/source_lower_machine_env.sh

sudo systemctl status joint-controller-supervisor.service --no-pager
ros2 node list --no-daemon --spin-time 5
ros2 topic echo /workspace/status \
  robot_control_msg/msg/WorkspaceStatus \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --once
```

应看到 supervisor 节点，workspace 默认为：

```text
state: STOPPED
mode: UNKNOWN
```

此时 EtherCAT、REAL stack 和电机都不应被自动启动。

## 9. 仿真验收

先在 Jetson 使用 SIM profile。不要在 Ubuntu 手动执行 `start.sh`：

```bash
robotctl down
robotctl up sim
robotctl start sim
robotctl status
```

或者使用：

```bash
robotctl ready sim
```

下位机检查：

```bash
systemctl status joint-controller-stack-sim.service --no-pager
systemctl status joint-controller-rviz.service --no-pager

ros2 topic hz /arm/joint_states
ros2 topic echo /arm_cartesian_path_execution_status \
  robot_control_msg/msg/CartesianExecutionStatus \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --once
```

仿真成功条件：

- workspace 为 `RUNNING + SIMULATION`；
- `/arm/joint_states` 持续发布且包含 14 个关节；
- motion/execution/TCP 反馈存在；
- RViz 在有图形会话时启动；
- EtherCAT 和 IGH 驱动没有启动。

停止顺序：

```bash
robotctl power off
robotctl stop
robotctl down
```

## 10. 真机验收

必须满足：物理急停可用、机械臂周围无人、双臂有支撑空间、供电和 EtherCAT 接线已检查。

### 10.1 启动但保持失能

Jetson：

```bash
robotctl down
robotctl up real --confirm-real
robotctl start real --confirm-real
robotctl status
```

首先确认：

```text
workspace=RUNNING mode=REAL
cartesian_servers=1
power=0/14 all_enabled=False
joint_states=14 joints
motion=moving:False
```

Ubuntu 下位机检查 EtherCAT：

```bash
sudo ethercat master
sudo ethercat slaves
```

成功条件：

- Master0 和 Master1 均为 `Phase: Operation`、`Active: yes`；
- 两个 master 各发现 7 个从站；
- 14 个从站全部为 `OP`；
- `Tx errors: 0`、`Lost frames: 0`；
- `igh_driver` 持续运行。

```bash
pgrep -af '/home/user/joint_controller/src/erobot_igh_driver/build/igh_driver'
sudo systemctl status joint-controller-stack-real.service --no-pager
```

### 10.2 模式、反馈和使能

Jetson：

```bash
robotctl power off
robotctl mode position
robotctl status
robotctl watch joints --once
```

关节位置必须是有限值、顺序完整、没有明显跳变。确认当前位置与控制器保持目标一致后，
只执行一次使能：

```bash
robotctl power on
robotctl status
```

必须看到：

```text
power=14/14 all_enabled=True
status_codes 全部为 39 (0x27, Operation enabled)
```

若不是 14/14，禁止发送运动命令。不要把 `33`、`64`、`8` 或默认 `0` 当作使能成功。

### 10.3 首次运动验收

先做单关节小增量：

```bash
robotctl joint ljoint7 0.005 \
  --relative \
  --vel 0.02 \
  --acc 0.02
```

再保存当前 TCP，并进行 1 mm 笛卡尔测试：

```bash
robotctl pose save real_deployment_baseline

robotctl cart-inc left \
  --x 0.001 \
  --vel 0.01 \
  --acc 0.02

robotctl cart-abs real_deployment_baseline \
  --vel 0.01 \
  --acc 0.02
```

每条命令必须等待最终返回；不要并发发送。Cartesian 成功必须同时有 Ubuntu 成功回执和
Jetson 新一代 TCP/execution 交叉确认。

### 10.4 真机停止

正常停止：

```bash
robotctl power off
robotctl stop
robotctl down
```

先确认 `power=0/14`，再停止 workspace。发生非预期运动时优先按物理急停。

## 11. 日志与诊断

### 11.1 systemd

```bash
sudo journalctl -u joint-controller-supervisor.service -n 150 --no-pager
sudo journalctl -u joint-controller-stack-sim.service -n 150 --no-pager
sudo journalctl -u joint-controller-stack-real.service -n 200 --no-pager
sudo journalctl -u joint-controller-rviz.service -n 100 --no-pager
```

实时跟踪：

```bash
sudo journalctl -fu joint-controller-supervisor.service
sudo journalctl -fu joint-controller-stack-real.service
```

### 11.2 运行日志

REAL 启动脚本使用：

```text
/home/user/joint_controller/log/runtime/igh_driver.log
/home/user/joint_controller/log/runtime/robot_arm.log
/home/user/joint_controller/log/runtime/robot_srv.log
```

```bash
tail -100 /home/user/joint_controller/log/runtime/igh_driver.log
tail -100 /home/user/joint_controller/log/runtime/robot_arm.log
tail -100 /home/user/joint_controller/log/runtime/robot_srv.log
```

### 11.3 常见问题

`Unit joint-controller-stack-real.service could not be found`：

- 确认命令是在 Ubuntu 下位机而不是 Jetson 上执行；
- 重新运行 `sudo ./scripts/install_workspace_control_services.sh`。

`service unavailable: /workspace/request_start_real`：

- Jetson real gateway/planner 未启动；
- 先执行 `robotctl down`、`robotctl up real --confirm-real`。

workspace 一直 `STOPPED/UNKNOWN`：

- 检查 supervisor journal；
- 检查 systemd unit 是否已安装；
- 确认上下位机 Domain、RMW、IP 和防火墙一致。

`power off` 超时：

- 若已经执行 `robotctl stop`，Ubuntu 电源服务已经退出；
- 正确顺序是 `power off -> stop`。

EtherCAT 为 `Idle/PREOP`：

- REAL stack 未启动时可以正常；
- REAL stack 启动后仍为该状态，则检查 IGH driver、网口绑定、从站供电和日志。

修改代码后现象不变：

- 重新编译实际运行的 Ubuntu 工作空间；
- `robotctl power off`、`robotctl stop` 后重新 start；
- 仅重启 Jetson gateway 不会替换 Ubuntu 已运行的进程。

重复节点或服务随机响应：

```bash
ros2 node list --no-daemon --spin-time 5
ros2 topic info /workspace/status --verbose
pgrep -af 'ros2 launch|workspace_supervisor|cartesian_single_control_srv'
```

确保 supervisor、控制栈和 gateway 各只有一个受管实例。

## 12. 更新与回滚流程

更新代码前：

```bash
robotctl power off
robotctl stop
robotctl down
```

Ubuntu 下位机更新并编译：

```bash
cd /home/user/joint_controller
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
sudo ./scripts/install_workspace_control_services.sh
```

如果只修改 C++ 并使用 symlink install，仍必须完整停止并重新启动 workspace，正在运行的
进程不会自动加载新二进制。回滚时使用已验证的代码版本重新编译并重复上述安装流程。

## 13. 最终验收清单

- [ ] 控制网口为 `192.168.2.20/24`，能 ping `192.168.2.10`。
- [ ] 上下位机时间同步，ROS Domain/RMW/DDS 配置一致。
- [ ] `robot_control_msg` 接口版本一致。
- [ ] 两个 EtherCAT master 绑定正确网口，各发现 7 个从站。
- [ ] supervisor 开机自启，SIM/REAL/RViz unit 均 disabled。
- [ ] 重启后 workspace 为 `STOPPED + UNKNOWN`，电机未使能。
- [ ] 仿真可进入 `RUNNING + SIMULATION` 并安全停止。
- [ ] REAL 启动后两个 master 为 active、14 个从站为 OP。
- [ ] 电机使能必须为 `14/14`、状态码全部 `39`。
- [ ] 单关节小幅运动和 1 mm Cartesian 往返通过交叉确认。
- [ ] `power off -> stop -> down` 完成后 0/14 电机使能。
- [ ] 物理急停经过独立安全流程验证。
