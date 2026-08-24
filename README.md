# Joint Controller 2.0

双臂机械臂 ROS2 控制系统，提供关节空间和笛卡尔空间的运动控制服务。

下位机独立通信功能包从 IP、DDS、构建、systemd 自启动到更换上位机的完整部署流程见
[docs/robot_lower_gateway_deployment.md](docs/robot_lower_gateway_deployment.md)。

Ubuntu 下位机的网络、ROS/EtherCAT、systemd 自启动和 SIM/REAL 验收流程见
[docs/ubuntu_lower_machine_deployment.md](docs/ubuntu_lower_machine_deployment.md)。

可移植下位机通信网关的包结构、独立部署和新机器人适配流程见
[docs/lower_gateway_migration.md](docs/lower_gateway_migration.md)。
复制功能包后的完整部署与使用说明也随包提供：
[src/robot_lower_gateway/README.md](src/robot_lower_gateway/README.md)。

整套系统的文档入口和 Jetson 上位机部署说明见
[docs/robot_system_deployment.md](docs/robot_system_deployment.md) 与
[docs/jetson_upper_machine_deployment.md](docs/jetson_upper_machine_deployment.md)。

## 项目结构

```
src/
├── robot_arm_description/      # 机器人 URDF 模型描述
├── robot_control/              # 高层控制服务（控制接口）
│   ├── launch/                 # 启动文件
│   └── src/                    # 控制服务实现
├── robot_control_msg/          # 消息和服务定义
│   ├── msg/                    # 消息类型
│   └── srv/                    # 服务类型
├── robot_lower_gateway/        # 可移植下位机网关完整功能包
├── erobot_controller/          # 底层 ros2_control 控制器
├── erobot_hw/                  # EtherCAT 硬件接口
└── thirdparty/                 # 第三方库 (Ruckig, qpOASES)
```

## 核心接口说明

本系统提供 5 个主要的 ROS2 服务用于机械臂控制：

---

### 1. 关节空间绝对位置控制

**服务名称**: `arm_absolute_control`

**服务类型**: `robot_control_msg/srv/JointAbsoluteControl`

**请求参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| ljoint1 ~ ljoint7 | float64 | 左臂 7 个关节的目标位置（弧度） |
| rjoint1 ~ rjoint7 | float64 | 右臂 7 个关节的目标位置（弧度） |
| vel | float64 | 关节速度限制（可选） |
| acc | float64 | 关节加速度限制（可选） |

**响应参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| success | bool | 是否成功 |
| message | string | 状态消息 |

**调用示例** (Python):
```python
import rclpy
from rclpy.node import Node
from robot_control_msg.srv import JointAbsoluteControl

def call_joint_control(node):
    client = node.create_client(JointAbsoluteControl, 'arm_absolute_control')

    req = JointAbsoluteControl.Request()
    req.ljoint1 = 0.0
    req.ljoint2 = 0.0
    req.ljoint3 = 0.0
    req.ljoint4 = 0.0
    req.ljoint5 = 0.0
    req.ljoint6 = 0.0
    req.ljoint7 = 0.0
    req.rjoint1 = 0.0
    req.rjoint2 = 0.0
    req.rjoint3 = 0.0
    req.rjoint4 = 0.0
    req.rjoint5 = 0.0
    req.rjoint6 = 0.0
    req.rjoint7 = 0.0
    req.vel = 0.5
    req.acc = 0.5

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return future.result()
```

**调用示例** (命令行):
```bash
ros2 service call /arm_absolute_control robot_control_msg/srv/JointAbsoluteControl "{ljoint1: 0.0, ljoint2: 0.0, ljoint3: 0.0, ljoint4: 0.0, ljoint5: 0.0, ljoint6: 0.0, ljoint7: 0.0, rjoint1: 0.0, rjoint2: 0.0, rjoint3: 0.0, rjoint4: 0.0, rjoint5: 0.0, rjoint6: 0.0, rjoint7: 0.0, vel: 0.5, acc: 0.5}"
```

---

### 2. 笛卡尔空间绝对位置控制

**服务名称**: `cartesian_absolute_control`

**服务类型**: `robot_control_msg/srv/CartesianAbsoluteControl`

**请求参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| lx, ly, lz | float64 | 左臂 TCP 目标位置 (米) |
| lroll, lpitch, lyaw | float64 | 左臂 TCP 欧拉角姿态 (弧度) |
| lqx, lqy, lqz, lqw | float64 | 左臂 TCP 四元数姿态 (若全为0则使用欧拉角) |
| rx, ry, rz | float64 | 右臂 TCP 目标位置 (米) |
| rroll, rpitch, ryaw | float64 | 右臂 TCP 欧拉角姿态 (弧度) |
| rqx, rqy, rqz, rqw | float64 | 右臂 TCP 四元数姿态 (若全为0则使用欧拉角) |
| vel | float64 | 运动速度限制（可选） |
| acc | float64 | 运动加速度限制（可选） |

**响应参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| success | bool | 是否成功 |
| message | string | 状态消息 |

**调用示例** (Python):
```python
from robot_control_msg.srv import CartesianAbsoluteControl

def call_cartesian_absolute(node):
    client = node.create_client(CartesianAbsoluteControl, 'cartesian_absolute_control')

    req = CartesianAbsoluteControl.Request()
    # 左臂目标位置
    req.lx = 0.3
    req.ly = 0.0
    req.lz = 0.5
    # 使用欧拉角表示姿态
    req.lroll = 0.0
    req.lpitch = 0.0
    req.lyaw = 0.0
    # 四元数设为0表示使用欧拉角
    req.lqx = 0.0
    req.lqy = 0.0
    req.lqz = 0.0
    req.lqw = 0.0

    # 右臂目标位置
    req.rx = 0.3
    req.ry = 0.0
    req.rz = 0.5
    req.rroll = 0.0
    req.rpitch = 0.0
    req.ryaw = 0.0
    req.rqx = 0.0
    req.rqy = 0.0
    req.rqz = 0.0
    req.rqw = 0.0

    req.vel = 0.5
    req.acc = 0.5

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return future.result()
```

**调用示例** (命令行):
```bash
ros2 service call /cartesian_absolute_control robot_control_msg/srv/CartesianAbsoluteControl "{lx: 0.3, ly: 0.0, lz: 0.5, lroll: 0.0, lpitch: 0.0, lyaw: 0.0, lqx: 0.0, lqy: 0.0, lqz: 0.0, lqw: 0.0, rx: 0.3, ry: 0.0, rz: 0.5, rroll: 0.0, rpitch: 0.0, ryaw: 0.0, rqx: 0.0, rqy: 0.0, rqz: 0.0, rqw: 0.0, vel: 0.5, acc: 0.5}"
```

---

### 3. 笛卡尔空间增量位置控制

**服务名称**: `cartesian_increment_control`

**服务类型**: `robot_control_msg/srv/CartesianIncrementControl`

**请求参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| lx, ly, lz | float64 | 左臂 TCP 相对于当前位置的增量 (米) |
| lroll, lpitch, lyaw | float64 | 左臂 TCP 相对于当前位置的姿态增量 (弧度) |
| lqx ~ lqw | float64 | 左臂 TCP 四元数增量 (若全为0则使用欧拉角) |
| rx, ry, rz | float64 | 右臂 TCP 增量 (米) |
| rroll, rpitch, ryaw | float64 | 右臂 TCP 姿态增量 (弧度) |
| rqx ~ rqw | float64 | 右臂 TCP 四元数增量 |
| vel | float64 | 运动速度限制 |
| acc | float64 | 运动加速度限制 |

**响应参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| success | bool | 是否成功 |
| message | string | 状态消息 |

**调用示例** (Python):
```python
from robot_control_msg.srv import CartesianIncrementControl

def call_cartesian_increment(node):
    client = node.create_client(CartesianIncrementControl, 'cartesian_increment_control')

    req = CartesianIncrementControl.Request()
    # 左臂增量移动 0.1米
    req.lx = 0.1
    req.ly = 0.0
    req.lz = 0.0
    req.lroll = 0.0
    req.lpitch = 0.0
    req.lyaw = 0.0
    req.lqx = 0.0
    req.lqy = 0.0
    req.lqz = 0.0
    req.lqw = 0.0

    # 右臂保持不变
    req.rx = 0.0
    req.ry = 0.0
    req.rz = 0.0
    req.rroll = 0.0
    req.rpitch = 0.0
    req.ryaw = 0.0
    req.rqx = 0.0
    req.rqy = 0.0
    req.rqz = 0.0
    req.rqw = 0.0

    req.vel = 0.5
    req.acc = 0.5

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return future.result()
```

---

### 4. 笛卡尔空间多路点绝对路径控制

**服务名称**: `cartesian_path_absolute_control`

**服务类型**: `robot_control_msg/srv/CartesianPathAbsoluteControl`

**请求参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| left_waypoints | geometry_msgs/Pose[] | 左臂绝对路径点序列 |
| left_blend_radii | float64[] | 左臂相邻路点间的圆角过渡半径 |
| right_waypoints | geometry_msgs/Pose[] | 右臂绝对路径点序列 |
| right_blend_radii | float64[] | 右臂相邻路点间的圆角过渡半径 |
| vel | float64 | 运动速度限制 |
| acc | float64 | 运动加速度限制 |

**响应参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| success | bool | 是否成功 |
| message | string | 状态消息 |

**调用示例** (Python):
```python
from geometry_msgs.msg import Pose
from robot_control_msg.srv import CartesianPathAbsoluteControl

def call_cartesian_path_control(node):
    client = node.create_client(CartesianPathAbsoluteControl, 'cartesian_path_absolute_control')

    req = CartesianPathAbsoluteControl.Request()

    # 定义左臂路径点
    pose1 = Pose()
    pose1.position.x = 0.3
    pose1.position.y = 0.2
    pose1.position.z = 0.4
    pose1.orientation.w = 1.0

    pose2 = Pose()
    pose2.position.x = 0.4
    pose2.position.y = 0.2
    pose2.position.z = 0.5
    pose2.orientation.w = 1.0

    req.left_waypoints = [pose1, pose2]
    req.left_blend_radii = [0.01]  # 第一个过渡半径
    req.right_waypoints = []
    req.right_blend_radii = []

    req.vel = 0.3
    req.acc = 0.3

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return future.result()
```

---

### 5. 笛卡尔空间多路点增量路径控制

**服务名称**: `cartesian_path_increment_control`

**服务类型**: `robot_control_msg/srv/CartesianPathIncrementControl`

**请求参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| left_waypoints | geometry_msgs/Pose[] | 左臂相对于当前点的增量路径点 |
| left_blend_radii | float64[] | 左臂圆角过渡半径 |
| right_waypoints | geometry_msgs/Pose[] | 右臂增量路径点 |
| right_blend_radii | float64[] | 右臂圆角过渡半径 |
| vel | float64 | 运动速度限制 |
| acc | float64 | 运动加速度限制 |

**响应参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| success | bool | 是否成功 |
| message | string | 状态消息 |

---

## 相关话题

### 订阅话题 (Inputs)

| 话题 | 类型 | 说明 |
|------|------|------|
| `/arm/joint_states` | sensor_msgs/JointState | 机械臂当前关节状态 |
| `/arm/arm_controller/motion_status` | robot_control_msg/ArmMotionStatus | 机械臂运动状态 |
| `/arm_tcp_pose` | robot_control_msg/EndEffectorPose | TCP 末端执行器位姿 |
| `/arm_cartesian_path_execution_status` | robot_control_msg/CartesianExecutionStatus | 路径执行状态 |

### 发布话题 (Outputs)

| 话题 | 类型 | 说明 |
|------|------|------|
| `/arm_joint_absolute_cmd` | robot_control_msg/Robotarmjoint | 关节绝对位置命令 |
| `/arm_cartrsian_position_cmd` | robot_control_msg/Robotarmmovel | 笛卡尔位置命令 |
| `/arm_cartesian_path_cmd` | robot_control_msg/Robotarmmovelpath | 笛卡尔路径命令 |

---

## 消息类型详解

### ArmMotionStatus.msg
```bash
bool is_moving    # 是否在运动中
bool goal_reached # 是否到达目标点
builtin_interfaces/Time stamp
```

### CartesianExecutionStatus.msg
```bash
builtin_interfaces/Time stamp

uint8 IDLE=0           # 空闲状态
uint8 PLANNING=1        # 规划中
uint8 EXECUTING=2       # 执行中
uint8 STREAM_FINISHED=3 # 流传输完成
uint8 FAILED=4          # 失败
uint8 REJECTED_BUSY=5   # 被拒绝（忙）

uint8 state                      # 当前状态
uint32 planned_points             # 规划点数
float64 planned_duration_sec      # 规划时长
string message                    # 状态消息
```

### EndEffectorPose.msg
```bash
geometry_msgs/Pose left_ee_pose   # 左臂末端位姿
geometry_msgs/Pose right_ee_pose  # 右臂末端位姿
```

---

## 启动方式

### 完整启动（需要 root 权限）
```bash
cd /home/user/joint_controller_2.0

```

### 单独启动控制服务
```bash
# 设置环境
source /path/to/workspace/install/setup.bash

# 启动关节空间控制
ros2 launch robot_control robot_control.launch.py
```

### 启动参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `urdf_path` | `robot_arm_description/urdf/right_left_arm.urdf` | URDF 文件路径 |
| `ee_frame_left` | `lee_link` | 左臂末端连杆名称 |
| `ee_frame_right` | `ree_link` | 右臂末端连杆名称 |
| `joint_state_topic` | `/arm/joint_states` | 关节状态话题 |
| `motion_status_topic` | `/arm/arm_controller/motion_status` | 运动状态话题 |

---

## 服务节点列表

| 节点名称 | 服务名称 | 功能 |
|---------|---------|------|
| `joint_absolute_control_srv` | `arm_absolute_control` | 关节空间绝对位置控制 |
| `cartesian_absolute_control_srv` | `cartesian_absolute_control` | 笛卡尔空间绝对位置控制 |
| `cartesian_increment_control_srv` | `cartesian_increment_control` | 笛卡尔空间增量位置控制 |
| `cartesian_path_absolute_control_srv` | `cartesian_path_absolute_control` | 多路点绝对路径控制 |
| `cartesian_path_increment_control_srv` | `cartesian_path_increment_control` | 多路点增量路径控制 |
| `cartesian_moveL_path` | - | MoveL 轨迹规划节点 |

---

## TCP 参数配置

系统支持动态配置 TCP (Tool Center Point) 偏移参数：

| 参数 | 说明 |
|------|------|
| `tcp_left_x/y/z` | 左臂 TCP 位置偏移 (米) |
| `tcp_left_roll/pitch/yaw` | 左臂 TCP 姿态偏移 (弧度) |
| `tcp_right_x/y/z` | 右臂 TCP 位置偏移 (米) |
| `tcp_right_roll/pitch/yaw` | 右臂 TCP 姿态偏移 (弧度) |

**示例** - 在 launch 文件中配置：
```python
Node(
    package="robot_control",
    executable="cartesian_absolute_control_srv",
    name="cartesian_absolute_control_srv",
    parameters=[{
        "urdf_path": "/path/to/urdf.urdf",
        "ee_frame_left": "lee_link",
        "ee_frame_right": "ree_link",
        "tcp_left_x": 0.1,
        "tcp_left_z": 0.05,
    }],
)
```

---

## 使用注意事项

1. **单位**: 关节空间使用弧度，笛卡尔空间使用米和弧度
2. **坐标系**: 笛卡尔位置基于 base_link 坐标系
3. **姿态表示**: 优先使用四元数，设置为全零时使用欧拉角 (ZYX 顺序)
4. **速度/加速度**: 值为相对比例 (0.0 ~ 1.0)
5. **路点Blend**: 相邻路点间的圆角半径，单位为米
6. **运动状态**: 建议在发送命令前检查 `motion_status` 确保机械臂处于静止状态

---

## 完整调用流程示例

```python
import rclpy
from rclpy.node import Node
from robot_control_msg.srv import JointAbsoluteControl
from robot_control_msg.msg import ArmMotionStatus
from rclpy.qos import QoSProfile

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller_client')
        self.joint_client = self.create_client(JointAbsoluteControl, 'arm_absolute_control')
        self.motion_status = None

        qos = QoSProfile(depth=10)
        self.create_subscription(ArmMotionStatus, '/arm/arm_controller/motion_status',
                                self.motion_callback, qos)

    def motion_callback(self, msg):
        self.motion_status = msg

    def wait_for_idle(self, timeout=10.0):
        # 等待机械臂停止
        ...

    def move_to_joints(self, left_joints, right_joints, vel=0.5, acc=0.5):
        req = JointAbsoluteControl.Request()
        joints = [left_joints, right_joints]
        for i, joint_list in enumerate(joints):
            prefix = 'ljoint' if i == 0 else 'rjoint'
            for j, val in enumerate(joint_list):
                setattr(req, f'{prefix}{j+1}', val)
        req.vel = vel
        req.acc = acc

        self.get_logger().info('Sending joint command...')
        future = self.joint_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        if result.success:
            self.get_logger().info(f'Motion started: {result.message}')
        else:
            self.get_logger().error(f'Motion failed: {result.message}')
        return result

def main(args=None):
    rclpy.init(args=args)
    controller = ArmController()

    # 移动到初始位置
    left = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    right = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    controller.move_to_joints(left, right)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 依赖

- ROS2 (Humble/Jazzy)
- pinocchio (运动学/动力学计算)
- ruckig (时间最优轨迹规划)
- qpOASES (二次规划)
- ros2_control
- EtherCAT 驱动


######
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
