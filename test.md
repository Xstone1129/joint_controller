# Lift and Waist Debug Guide

本文档基于当前工作空间：

```bash
/home/user/joint_controller
```

## 0. Lift safety and ownership

Lift 是垂直升降轴。真机测试前必须：

- 清空机械运动区域，安排人员监护。
- 准备物理急停、STO 和防坠措施。
- 第一次运动从 1 mm、10 rpm 以内开始。
- 不要把软件 `/joint/safety/estop` 当作安全认证急停。

以下三种方式不能同时运行：

1. `tools/run_lift_ethercat_cli.sh` 直接占用 Master2。
2. 独立的 ROS lift launch。
3. `joint-controller-stack-real.service` 完整机器人控制栈。

启动 ROS 前先退出 CLI。启动完整 systemd 控制栈时，不要再手动启动第二个 lift launch。

---

# 一、当前 Lift 接口

Lift 使用自定义 `joint_hardware/LiftController`，不是 waist 使用的
`FollowJointTrajectory` action。

```text
controller manager:       /lift/controller_manager
controller:               lift_controller

position service:         /joint/lift/command
trajectory topic:         /joint/lift/trajectory
stream topic:             /joint/lift/stream
jog topic:                /joint/lift/jog_velocity
stop service:             /joint/lift/stop
hold service:             /joint/lift/hold

driver status:            /joint/lift/driver_status
controller status:        /joint/lift/control_status
joint state:              /joint_states

brake service:             /lift_brake_command
host zero service:         /lift_reset_zero
drive homing service:      /lift_home
drive zero service:        /lift_set_drive_zero

software estop:            /joint/safety/estop
software estop reset:      /joint/safety/reset
```

当前 lift 的软件范围：

```text
ROS position range:       [-1.0, 0.0] m
joint name:               joint_motor
lead:                     10.0 mm/rev
command units/rev:        10000
encoder counts/rev:       131072
ROS maximum speed:        0.060 m/s
ROS maximum acceleration: 0.10 m/s^2
```

ROS 正方向是位置增大，通常表示向上、位置趋近 `0.0 m`。
ROS 负方向是位置减小，通常表示向下、位置趋近 `-1.0 m`。

---

# 二、每个 ROS 终端的环境

所有 ROS 命令都建议先执行：

```bash
cd /home/user/joint_controller
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/user/joint_controller/cyclonedds.xml

unset ROS_DISCOVERY_SERVER
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset FASTDDS_DEFAULT_PROFILES_FILE
unset FASTDDS_BUILTIN_TRANSPORTS
```

---

# 三、确认 EtherCAT 和从站参数

## 3.1 确认 Master2

Lift 当前使用 Master2，不能使用 Master0：

```bash
sudo ethercat master
sudo ethercat slaves -m 2
sudo ethercat slaves -m 2 -v
sudo ethercat pdos -m 2 -p 0
```

也可以运行工程内 preflight：

```bash
sudo /home/user/joint_controller/install/joint_hardware/lib/joint_hardware/lift_ethercat_preflight.sh \
  2 0 enp5s0
```

输出中应确认：

```text
slave_position:=0
slave_vendor_id:=0x00004321
slave_product_code:=0x0000000a
```

如果现场输出不同，以现场 preflight 输出为准，不要盲目复制上面的 ID。

## 3.2 检查网卡和 EtherLab 配置

```bash
ip link show enp5s0
ls -l /dev/EtherCAT2

sudo /home/user/joint_controller/install/joint_hardware/lib/joint_hardware/lift_ethercat_host_setup.sh \
  enp5s0 --check 2
```

只有确认 Master2 没有绑定 `enp5s0` 时，才执行配置应用：

```bash
sudo /home/user/joint_controller/install/joint_hardware/lib/joint_hardware/lift_ethercat_host_setup.sh \
  enp5s0 --apply 2
```

应用配置后重新检查：

```bash
ls -l /dev/EtherCAT2
sudo ethercat master
sudo ethercat slaves -m 2
```

---

# 四、启动方式 A：完整真机 ROS 栈

如果平时使用工程的完整机器人控制栈，推荐使用：

```bash
sudo systemctl restart joint-controller-stack-real.service
```

该服务会负责：

- 启动 EtherCAT。
- 检查 Master0、Master1、Master2。
- 启动机械臂驱动。
- 启动 lift EtherCAT ROS 控制器。
- 启动其它机器人控制节点。

确认服务状态：

```bash
systemctl status joint-controller-stack-real.service --no-pager
ros2 node list | grep -E 'lift|controller_manager'
ros2 control list_controllers -c /lift/controller_manager
```

完整栈运行时，不要再执行下面的独立 lift launch。

---

# 五、启动方式 B：只启动 Lift ROS 控制器

适用于只调试 lift，不启动完整机械臂控制栈的情况。

## 5.1 启动前检查进程占用

```bash
systemctl is-active joint-controller-stack-real.service
ros2 node list | grep -E 'lift|controller_manager'
```

如果完整栈正在运行，应使用启动方式 A，或者先停止完整栈。

CLI 也必须退出。正常情况下，在 CLI 窗口按 `q` 退出即可。确认没有残留进程：

```bash
pgrep -af lift_ethercat_cli || true
```

如果仍有残留进程，应先确认进程归属，再由现场人员手动结束对应进程。

## 5.2 只读启动

只读启动不授权释放抱闸，不允许运动。用于检查 ROS、EtherCAT 和反馈。

```bash
ros2 launch joint_hardware lift_ethercat.launch.py \
  backend:=etherlab \
  namespace:=lift \
  master_index:=2 \
  slave_alias:=0 \
  slave_position:=0 \
  slave_vendor_id:=0x00004321 \
  slave_product_code:=0x0000000a \
  ethercat_interface:=enp5s0 \
  cycle_ms:=10 \
  expected_working_counter:=1 \
  command_units_per_rev:=10000 \
  encoder_counts_per_rev:=131072 \
  lead_mm_per_rev:=10.0 \
  lift_sign:=-1.0 \
  position_min_m:=-1.0 \
  position_max_m:=0.0 \
  max_rpm:=360 \
  brake_control_enabled:=false \
  brake_release_wait_ms:=-1 \
  limit_switch_enabled:=false
```

## 5.3 真机运动启动

当前工程验证过的 lift 参数：

```text
P04.37=150 ms
P04.38=0 ms
P04.39=30 rpm
P06.14=500 ms
P05.06=3
P05.10=4
```

垂直抱闸参数必须以现场驱动器实际确认值为准。确认一致后，可以使用：

```bash
mkdir -p /home/user/.local/state/joint_controller

ros2 launch joint_hardware lift_ethercat.launch.py \
  backend:=etherlab \
  namespace:=lift \
  master_index:=2 \
  slave_alias:=0 \
  slave_position:=0 \
  slave_vendor_id:=0x00004321 \
  slave_product_code:=0x0000000a \
  ethercat_interface:=enp5s0 \
  cycle_ms:=10 \
  expected_working_counter:=1 \
  command_units_per_rev:=10000 \
  encoder_counts_per_rev:=131072 \
  lead_mm_per_rev:=10.0 \
  lift_sign:=-1.0 \
  position_min_m:=-1.0 \
  position_max_m:=0.0 \
  max_rpm:=360 \
  brake_control_enabled:=true \
  brake_release_wait_ms:=0 \
  brake_p04_37_ms:=150 \
  brake_p04_39_rpm:=30 \
  brake_p06_14_ms:=500 \
  brake_p05_06_mode:=3 \
  brake_p05_10_mode:=4 \
  limit_switch_enabled:=false \
  use_persistent_zero_offset:=true \
  zero_offset_file:=/home/user/.local/state/joint_controller/lift_zero_offset.cfg
```

如果实际 vendor/product、P04.38 或 PDO 参数不同，必须替换为现场确认值。

---

# 六、启动后检查

## 6.1 检查 controller

注意必须指定 lift 的 controller manager：

```bash
ros2 control list_controllers -c /lift/controller_manager
```

预期：

```text
joint_state_broadcaster  ...  active
lift_controller          ...  active
```

## 6.2 检查硬件接口

```bash
ros2 control list_hardware_interfaces \
  -c /lift/controller_manager
```

至少应存在：

```text
command interfaces
joint_motor/acceleration
joint_motor/position
joint_motor/power_enable
joint_motor/velocity

state interfaces
joint_motor/position
joint_motor/velocity
joint_motor/status
joint_motor/error_code
joint_motor/mode
joint_motor/brake_unlocked
joint_motor/power_enable
```

## 6.3 检查驱动状态

```bash
ros2 topic echo /joint/lift/driver_status --once --full-length
```

正常运动前建议确认：

```text
link_state:             operational
cia402_state:           operation_enabled
error_code:             0
mode:                   9
ethercat_operational:   true
working_counter_ok:     true
feedback_fresh:         true
pdo_fresh:              true
estop_latched:          false
motion_blocked:         false
```

## 6.4 检查控制器状态

```bash
ros2 topic echo /joint/lift/control_status --once --full-length
```

重点字段：

```text
mode
position
command_position
velocity
brake_gate_ready
power_enable_command
power_enabled
trajectory_active
jog_active
estop_latched
```

## 6.5 检查 JointState

当前配置显式设置 `use_local_topics: false`，应该发布全局话题：

```bash
ros2 param get /lift/joint_state_broadcaster use_local_topics
ros2 topic echo /joint_states --once --full-length
```

预期能看到 `joint_motor`：

```text
name:
- joint_motor
position:
- ...
```

---

# 七、释放抱闸和使能

只有确认 EtherCAT、PDO、CiA402 和安全状态正常后执行：

```bash
ros2 service call /lift_brake_command \
  std_srvs/srv/SetBool "{data: true}"
```

成功响应通常为：

```text
Operation enabled requested; brake unlock and P04.38 release timing are checked before motion
```

再次查看状态：

```bash
ros2 topic echo /joint/lift/driver_status --once --full-length
```

注意：

- `brake_control_enabled:=false` 时，`data: true` 会被拒绝。
- `data: false` 是始终允许的安全锁定操作。
- 发过 `data: false` 后，下一次运动前建议再次显式发 `data: true`。
- `accepted` 或 `success` 只表示命令被接收，不代表电机已经完成运动。

---

# 八、低速位置控制

## 8.1 相对移动 1 mm

向 ROS 正方向移动 1 mm：

```bash
ros2 service call /joint/lift/command \
  robot_control_msg/srv/SelectedJointControl \
  "{joint_names: [joint_motor], values: [0.001], relative: true, vel: 0.001, acc: 0.01}"
```

向 ROS 负方向移动 1 mm：

```bash
ros2 service call /joint/lift/command \
  robot_control_msg/srv/SelectedJointControl \
  "{joint_names: [joint_motor], values: [-0.001], relative: true, vel: 0.001, acc: 0.01}"
```

参数单位：

```text
values: 相对位置，单位 m
vel:    速度，单位 m/s
acc:    加速度，单位 m/s^2
```

## 8.2 绝对位置移动

例如移动到 `-0.050 m`：

```bash
ros2 service call /joint/lift/command \
  robot_control_msg/srv/SelectedJointControl \
  "{joint_names: [joint_motor], values: [-0.050], relative: false, vel: 0.005, acc: 0.01}"
```

绝对位置必须在：

```text
[-1.0, 0.0] m
```

发送命令后观察：

```bash
ros2 topic echo /joint/lift/control_status --once --full-length
ros2 topic echo /joint_states --once --full-length
```

---

# 九、连续点动

10 rpm 对应约 `0.0016667 m/s`。

## 9.1 向上点动

```bash
ros2 topic pub -r 50 /joint/lift/jog_velocity \
  std_msgs/msg/Float64 "{data: 0.0016667}"
```

按 `Ctrl+C` 停止发布后，补发零速度和停止命令：

```bash
ros2 topic pub --once /joint/lift/jog_velocity \
  std_msgs/msg/Float64 "{data: 0.0}"

ros2 service call /joint/lift/stop \
  std_srvs/srv/Trigger "{}"
```

## 9.2 向下点动

```bash
ros2 topic pub -r 50 /joint/lift/jog_velocity \
  std_msgs/msg/Float64 "{data: -0.0016667}"
```

## 9.3 键盘点动

```bash
ros2 run joint_hardware lift_keyboard_teleop --ros-args \
  -p speed_rpm:=10.0 \
  -p command_rate_hz:=50.0 \
  -p release_timeout_ms:=1000 \
  -p arm_timeout_ms:=1500
```

键盘：

```text
E       请求使能/释放抱闸
D       请求停机并锁抱闸
↑       向 ROS 正方向点动
↓       向 ROS 负方向点动
Space   停止并保持
Q       停止并退出
```

`release_timeout_ms` 是键盘程序判定“方向键已松开”的超时，不是 EtherCAT 周期。
不要设为 `150`：普通终端方向键的首次自动重复通常晚于 150 ms，程序会在抱闸完成
释放前发布零速度，控制器随即执行 STOP 并锁抱闸。该参数必须至少为 `600`，建议
使用 `1000`；松开方向键约 1 秒后停止并锁抱闸是预期的失效保护行为。

如果键盘提示 `waiting for joint_motor feedback`，检查：

```bash
ros2 topic echo /joint_states --once --full-length
ros2 topic list | grep joint_states
```

---

# 十、停止、抱闸和急停

## 10.1 普通停止

```bash
ros2 service call /joint/lift/stop \
  std_srvs/srv/Trigger "{}"
```

## 10.2 锁定抱闸

```bash
ros2 service call /lift_brake_command \
  std_srvs/srv/SetBool "{data: false}"
```

确认停止状态：

```bash
ros2 topic echo /joint/lift/driver_status --once --full-length
ros2 topic echo /joint/lift/control_status --once --full-length
```

## 10.3 软件急停

```bash
ros2 service call /joint/safety/estop \
  std_srvs/srv/Trigger "{}"
```

复位前必须确认机械和驱动状态安全：

```bash
ros2 service call /joint/safety/reset \
  std_srvs/srv/Trigger "{}"
```

软件 Quick Stop 不是安全认证急停，不能替代物理急停、STO 和防坠装置。

---

# 十一、Lift 设零和回零

## 11.1 Host ROS 零点

该操作把当前驱动反馈位置保存为 host 侧 ROS 零点，不写驱动器 P00.15，也不执行 HM。

只有当前机械位置已经确认是零点，并且电机静止时执行：

```bash
ros2 service call /joint/lift/stop \
  std_srvs/srv/Trigger "{}"

ros2 service call /lift_reset_zero \
  std_srvs/srv/Trigger "{}"
```

成功后零点文件默认位于：

```text
/home/user/.local/state/joint_controller/lift_zero_offset.cfg
```

`brake_control_enabled` 必须为 `true`，PDO 必须新鲜，速度必须接近零。

## 11.2 驱动器 HM 回零

该操作会执行驱动器 Homing Method 19，具有真实机械运动风险：

```bash
ros2 service call /lift_home \
  std_srvs/srv/Trigger "{}"
```

确认 HM 参数、方向、速度、机械限位和防坠措施后才能使用。

## 11.3 写驱动器零点

```bash
ros2 service call /lift_set_drive_zero \
  std_srvs/srv/Trigger "{}"
```

该接口会写驱动器侧零点，通常需要驱动器断电重启。不要把它和
`/lift_reset_zero` 混用。

## 11.4 禁止使用的调试接口

```bash
/lift_reset_velocity
```

该调试接管接口当前默认禁用，不用于正常运动测试。

---

# 十二、安全关机

```bash
ros2 service call /joint/lift/stop \
  std_srvs/srv/Trigger "{}"

ros2 service call /lift_brake_command \
  std_srvs/srv/SetBool "{data: false}"

ros2 topic echo /joint/lift/driver_status --once --full-length
```

确认实际速度接近零、目标速度为零、驱动已退出 Operation Enabled、抱闸状态已闭合后，再在 launch 终端按 `Ctrl+C`。

如果使用完整 systemd 栈，停止整个机器人栈：

```bash
sudo systemctl stop joint-controller-stack-real.service
```

---

# 十三、Lift 故障排查

## 13.1 controller 不 active

```bash
ros2 control list_controllers \
  -c /lift/controller_manager

ros2 node list | grep -E 'lift|controller_manager'
```

不要使用不带 `-c` 的旧命令：

```bash
ros2 control list_controllers
```

## 13.2 没有 `/joint_states`

```bash
ros2 param get /lift/joint_state_broadcaster use_local_topics
ros2 topic list | grep joint_states
ros2 node info /lift/joint_state_broadcaster
```

当前配置要求：

```text
use_local_topics: False
```

修改 YAML、launch 或 xacro 后，必须重新构建并重启 launch：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select joint_hardware --symlink-install
source install/setup.bash
```

## 13.3 命令 accepted 但电机不动

```bash
ros2 topic echo /joint/lift/driver_status --once --full-length
ros2 topic echo /joint/lift/control_status --once --full-length
```

重点检查：

```text
brake_gate_ready
power_enable_command
power_enabled
brake_unlocked
motion_blocked
estop_latched
trajectory_active
jog_active
target_rpm
```

确认：

- launch 使用 `backend:=etherlab`。
- launch 使用 `master_index:=2`。
- `brake_control_enabled:=true`。
- 已执行 `/lift_brake_command "{data: true}"`。
- 目标位置在 `[-1.0, 0.0]`。
- 没有 CLI、完整栈或其他节点同时占用 Master2。

## 13.4 EtherCAT 不是 operational

```bash
ip link show enp5s0
ls -l /dev/EtherCAT2
sudo ethercat master
sudo ethercat slaves -m 2
sudo ethercat slaves -m 2 -v
```

确认 EtherLab Master2 配置和从站位置为 `0`。

## 13.5 启动后出现旧参数

`ros2 launch` 使用的是 `install/joint_hardware/share/joint_hardware` 下的安装文件。
修改源码后的 YAML、launch 或 xacro 必须重新构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select joint_hardware --symlink-install
source install/setup.bash
```

---

# 十四、CLI 直控模式

CLI 与 ROS 互斥，只能二选一：

```bash
cd /home/user/joint_controller
sudo ./tools/run_lift_ethercat_cli.sh
```

快捷键：

```text
E       使能/保持
D       去使能
Z       将当前编码器位置设为 CLI 零点
U       向上移动约 10 mm
J       向下移动约 10 mm
Space   停止
Q       退出
```

退出 CLI 后，才能启动 ROS lift 控制器。

---

# 十五、Waist 电机

以下保留 waist 的独立调试流程。Lift 不使用 waist 的 action。

## 15.1 启动 waist

```bash
cd /home/user/joint_controller
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch joint_hardware waist_hardware.launch.py
```

等待日志出现：

```text
joint_hardware waist activated
Configured and activated waist_controller
```

## 15.2 检查 waist 状态

```bash
ros2 topic echo /joint/waist/driver_status --once
```

确认：

```text
initialized: true
fault_active: false
feedback_fresh: true
```

## 15.3 Waist 发送绝对位置

移动到 `0.750 rad`，3 秒完成：

```bash
ros2 action send_goal \
  /waist_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
    joint_names: [joint_qugan],
    points: [{
      positions: [0.750],
      velocities: [0.0],
      accelerations: [0.0],
      time_from_start: {sec: 3, nanosec: 0}
    }]
  }}"
```

观察实际位置：

```bash
ros2 topic echo /joint_states
```

回到测试前约 `0.738 rad`：

```bash
ros2 action send_goal \
  /waist_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
    joint_names: [joint_qugan],
    points: [{
      positions: [0.738],
      velocities: [0.0],
      accelerations: [0.0],
      time_from_start: {sec: 3, nanosec: 0}
    }]
  }}"
```

Waist 位置单位是弧度，允许范围为 `[0.0, 1.57] rad`。
首次运动建议保持在当前位置附近，不要直接测试行程端点。
