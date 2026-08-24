#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
# from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import casadi          
import numpy as np
import pinocchio as pin
from pathlib import Path
from pinocchio import casadi as cpin
from robot_control_msg.msg import Robotarmservomsg
from robot_control_msg.msg import Robotteleopctrl
from robot_control_msg.msg import Robotteleopstate
# from builtin_interfaces.msg import Duration
from rclpy.duration import Duration


def get_robot_arm_urdf_path():
    return str(
        (Path(get_package_share_directory("robot_arm_description")) / "urdf" / "right_left_arm.urdf")
    )
class Kinematics:
    def __init__(self, ee_frame_r,ee_frame_l) -> None:
        if isinstance(ee_frame_r, str):
            self.right_frame_name = ee_frame_r
            self.left_frame_name = ee_frame_l
        else:
            raise ValueError("ee_frame参数必须是字符串或包含至少两个元素的列表/元组")

    def buildFromURDF(self, urdf_file):
        self.arm = pin.RobotWrapper.BuildFromURDF(urdf_file)
        self.createSolver()

    def createSolver(self):
        self.model = self.arm.model
        self.data = self.arm.data

        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.model)
        self.cdata = self.cmodel.createData()

        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.model.nq, 1)

        print("q关节的数量", self.model.nq)

        self.cTf_l = casadi.SX.sym("tf_l", 4, 4)
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)

        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)
        
        # Get the hand joint ID and define the error function
        self.ee_id_r = self.model.getFrameId(self.right_frame_name)
        self.ee_id_l = self.model.getFrameId(self.left_frame_name)

        print("ee_id_r—————ee_id_l:   ", self.ee_id_r, self.ee_id_l)
        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_r,self.cTf_l],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.ee_id_r].translation - self.cTf_r[:3,3],
                    self.cdata.oMf[self.ee_id_l].translation - self.cTf_l[:3,3]
                )
            ],
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_r,self.cTf_l],
            [
                casadi.vertcat(
                    cpin.log3(self.cdata.oMf[self.ee_id_r].rotation @ self.cTf_r[:3,:3].T),
                    cpin.log3(self.cdata.oMf[self.ee_id_l].rotation @ self.cTf_l[:3,:3].T)
                )
            ],
        )

        # Defining the optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.model.nq)
        self.var_q_last = self.opti.parameter(self.model.nq)   # for smooth
        self.param_tf_r = self.opti.parameter(4, 4)
        self.param_tf_l = self.opti.parameter(4, 4)
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf_r,self.param_tf_l))
        self.rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf_r,self.param_tf_l))
        self.regularization_cost = casadi.sumsqr(self.var_q)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)

        # Setting optimization constraints and goals
        self.opti.subject_to(self.opti.bounded(
            self.model.lowerPositionLimit,
            self.var_q,
            self.model.upperPositionLimit)
        )
        self.opti.minimize(80.0 * self.translational_cost + 20*self.rotation_cost + 0.1 * self.regularization_cost + 1.0 * self.smooth_cost)

        ##### IPOPT #####
        opts = {
            'ipopt':{
                'print_level': 0,
                'max_iter': 20,
                'tol': 1e-5,
            },
            'print_time':True,# print or not
            'calc_lam_p':False # https://github.com/casadi/casadi/wiki/FAQ:-Why-am-I-getting-%22NaN-detected%22in-my-optimization%3F
        }
        self.opti.solver("ipopt", opts)

        self.init_data = np.zeros(self.model.nq)
      
    def ik(self, T_r, T_l, current_arm_motor_q=None, current_arm_motor_dq=None):
        if current_arm_motor_q is not None:
            self.init_data = current_arm_motor_q
        self.opti.set_initial(self.var_q, self.init_data)

        self.opti.set_value(self.param_tf_r, T_r)
        self.opti.set_value(self.param_tf_l, T_l)
        self.opti.set_value(self.var_q_last, self.init_data) # for smooth

        try:
            sol = self.opti.solve()
            # sol = self.opti.solve_limited()

            sol_q = self.opti.value(self.var_q)
            # self.smooth_filter.add_data(sol_q)
            # sol_q = self.smooth_filter.filtered_data
            print(f"sol_q:{sol_q} \ncurrent_arm_motor_q: \n{current_arm_motor_q} \n") 
            # print

            self.init_data = sol_q

            
            info = {
                "success": True,
                "iterations": sol.stats().get("iter_count", 0),  # 添加迭代次数信息
                "cost": float(sol.value(10 * self.translational_cost + 1.0*self.rotation_cost))
            }

            dof = np.zeros(self.model.nq)
            dof[:len(sol_q)] = sol_q
            return dof, info
        
        except Exception as e:
            print(f"ERROR in convergence: {e}")

            sol_q = self.opti.debug.value(self.var_q)

            print(f"sol_q:{sol_q} \ncurrent_arm_motor_q: \n{current_arm_motor_q} \n")

            info = {
                "success": False,
                "message": str(e),
                "iterations": 0  # 失败时迭代次数为0
            }

            dof = np.zeros(self.model.nq)
            dof[:len(sol_q)] = self.init_data
            
            return dof, info
    def debug_forward_kinematics(self, q=None):
        """
        调试函数：计算并打印当前配置下的正向运动学
        """
        if q is None:
            q = self.init_data
        
        # 确保 q 是 NumPy 数组，而不是列表
        if isinstance(q, list):
            q = np.array(q)
        
        print(f"\n=== 正向运动学调试 ===")
        print(f"关节角度 q: {q}")
        print(f"关节角度类型: {type(q)}")
        print(f"关节数量: {self.model.nq}")
        # print(f"末端执行器帧ID: {self.ee_id}")
        
        # 检查 ee_id 是否有效
        # if self.ee_id >= len(self.data.oMi):
        #     print(f"错误: ee_id {self.ee_id} 超出范围 (最大索引: {len(self.data.oMi)-1})")
        #     print(f"模型帧数量: {len(self.model.frames)}")
        #     print(f"模型帧名称: {[frame.name for frame in self.model.frames]}")
        #     return None, None
        
        try:
            # 计算正向运动学 - 确保 q 是 NumPy 数组
            pin.forwardKinematics(self.model, self.data, q)

            # 打印每个关节的位置
            # print("各关节位置:")
            # for i, (name, oMi) in enumerate(zip(self.model.names, self.data.oMi)):
            #     print(f"{i}: {name:<24} : {oMi.translation[0]:.2f} {oMi.translation[1]:.2f} {oMi.translation[2]:.2f}")
            # 打印每个连杆的位置
            # print("各连杆位置:")
            # for i, frame in enumerate(self.model.frames):
            #     frame_name = frame.name
            #     frame_placement = self.data.oMf[i]
            #     print(f"{i}: {frame_name:<24} : {frame_placement.translation[0]:.2f} {frame_placement.translation[1]:.2f} {frame_placement.translation[2]:.2f}")

            # 获取工具坐标系(末端执行器)的frame ID
            right_frame_id = self.model.getFrameId(self.right_frame_name)
            left_frame_id = self.model.getFrameId(self.left_frame_name)

            # # 更新frame的几何位置
            pin.updateFramePlacement(self.model, self.data, right_frame_id)
            pin.updateFramePlacement(self.model, self.data, left_frame_id)

            # 提取末端执行器的位置和旋转
            ee_pose_right = self.data.oMf[right_frame_id]
            ee_pose_left = self.data.oMf[left_frame_id]

            position_right = ee_pose_right.translation
            position_left = ee_pose_left.translation

            rotation_right = ee_pose_right.rotation
            rotation_left = ee_pose_left.rotation

            # 将旋转矩阵转换为RPY角
            rpy_right = pin.rpy.matrixToRpy(rotation_right)
            rpy_left = pin.rpy.matrixToRpy(rotation_left)

            print(f"末端位置: right:{position_right},left:{position_left}")
            # print(f"末端旋转矩阵:\n{rotation_right}")
            print(f"末端RPY角: right:{rpy_right},left:{rpy_left}")
            print("==============================\n")
            print([position_right,position_left], [rpy_right,rpy_left])
            # 返回位置和RPY角
            return [position_right,position_left], [rpy_right,rpy_left]

        except Exception as e:
            print(f"正向运动学计算错误: {e}")
            print(f"q 的形状: {q.shape if hasattr(q, 'shape') else '无形状属性'}")
            print(f"q 的类型: {type(q)}")
            return None, None



class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')

        self.ik =  Kinematics("ree_link","lee_link")
        self.ik.buildFromURDF(get_robot_arm_urdf_path())
        self.sorted_positions =np.zeros(self.ik.model.nq, dtype=np.float64)
                # 创建订阅者，订阅/joint_state话题，消息类型为JointState
        self.subscription = self.create_subscription(
            JointState,
            '/arm/joint_states',
            self.listener_callback,
            1  # 队列大小
        )

        self.twist_sub_ = self.create_subscription(
            Robotteleopctrl,
            "/arm_teleop_ctrl",
            self.twist_callback,
            1
        )
        self.command_publisher_ = self.create_publisher(
            Robotarmservomsg,
            "/arm_axis_position_cmd",
            1
        )

        self.robot_teleop_arm_state_publisher_ = self.create_publisher(
            Robotteleopstate,
            "/robot_teleop_arm_state",
            1
        )

        self.current_pos = [[0,0,0],[0,0,0]]
        self.current_rpy = [[0,0,0],[0,0,0]]
        
        self.subscription  # 防止未使用变量警告
    def listener_callback(self, msg):
        # 定义您关心的关节顺序
        # desired_joints = ['rjoint1', 'rjoint2', 'rjoint3', 'rjoint4', 'rjoint5', 'rjoint6', 'rjoint7'
        #                     ,'ljoint1', 'ljoint2', 'ljoint3', 'ljoint4', 'ljoint5', 'ljoint6', 'ljoint7']
        desired_joints = ['ljoint1', 'ljoint2', 'ljoint3', 'ljoint4', 'ljoint5', 'ljoint6', 'ljoint7',
                            'rjoint1', 'rjoint2', 'rjoint3', 'rjoint4', 'rjoint5', 'rjoint6', 'rjoint7']
        # 打印收到的关节信息
        # self.get_logger().info('收到关节状态:')
        
        # 提取关节名称、位置、速度、力矩
        joint_names = msg.name
        positions = msg.position
        
        # 创建一个字典来存储关节数据，键为关节名称
        joint_data = {}
        for i, name in enumerate(joint_names):
            pos = positions[i] if i < len(positions) else 0.0
            joint_data[name] = pos
        
        # 更新排序后的位置数组，而不是重新创建
        for i, joint_name in enumerate(desired_joints):
            if joint_name in joint_data:
                pos = joint_data[joint_name]
                self.sorted_positions[i] = pos
                
                # 格式化输出
                # pos_str = f'{pos:.2f} rad'
                # self.get_logger().info(f'关节: {joint_name}, 位置: {pos_str}')
            else:
                # 如果关节不存在，使用默认值（例如0.0）
                self.sorted_positions[i] = 0.0
                self.get_logger().warning(f'关节 {joint_name} 未找到数据，使用默认值 0.0')
        
        [self.current_pos[0],self.current_pos[1]],[self.current_rpy[0],self.current_rpy[1]] = self.ik.debug_forward_kinematics(self.sorted_positions)

        state_msg = Robotteleopstate()
        state_msg.right_arm_twist_state.linear.x = self.current_pos[0][0]
        state_msg.right_arm_twist_state.linear.y = self.current_pos[0][1]
        state_msg.right_arm_twist_state.linear.z = self.current_pos[0][2]
        state_msg.right_arm_twist_state.angular.x = self.current_rpy[0][0]
        state_msg.right_arm_twist_state.angular.y = self.current_rpy[0][1]
        state_msg.right_arm_twist_state.angular.z = self.current_rpy[0][2]
        state_msg.left_arm_twist_state.linear.x = self.current_pos[1][0]
        state_msg.left_arm_twist_state.linear.y = self.current_pos[1][1]
        state_msg.left_arm_twist_state.linear.z = self.current_pos[1][2]
        state_msg.left_arm_twist_state.angular.x = self.current_rpy[1][0]
        state_msg.left_arm_twist_state.angular.y = self.current_rpy[1][1]
        state_msg.left_arm_twist_state.angular.z = self.current_rpy[1][2]

        self.robot_teleop_arm_state_publisher_.publish(state_msg)


        # 现在 self.sorted_positions 包含14个按照 rjoint1 到 rjoint7 顺序排列的位置值还有ljoint1 到 ljoint7

    # 现在您可以在类的其他方法中使用 self.sorted_positions 数组
    def rpy_to_rotation_matrix(self, roll, pitch, yaw):
        # 绕X轴旋转（Roll）
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        # 绕Y轴旋转（Pitch）
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        # 绕Z轴旋转（Yaw）
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # 合并旋转矩阵
        R = Rz @ Ry @ Rx
        
        return R
    def publish_command(self, rqres):
        # 创建消息对象
        command_msg = Robotarmservomsg()
        if len(rqres) >= 14:
            command_msg.ljoint1_position = rqres[0]
            command_msg.ljoint2_position = rqres[1]
            command_msg.ljoint3_position = rqres[2]
            command_msg.ljoint4_position = rqres[3]
            command_msg.ljoint5_position = rqres[4]
            command_msg.ljoint6_position = rqres[5]
            command_msg.ljoint7_position = rqres[6]
            command_msg.rjoint1_position = rqres[7]
            command_msg.rjoint2_position = rqres[8]
            command_msg.rjoint3_position = rqres[9]
            command_msg.rjoint4_position = rqres[10]
            command_msg.rjoint5_position = rqres[11]
            command_msg.rjoint6_position = rqres[12]
            command_msg.rjoint7_position = rqres[13]
        else:
            self.get_logger().error(f"逆解结果长度不足: {len(rqres)},需要14个值")
            return

        command_msg.robot_power = True
        command_msg.run_time = 0.0
        command_msg.run_mode = 1

        # 设置时间戳为当前 ROS 时间 + 10 ms
        now = self.get_clock().now()
        delta = Duration(seconds=0, nanoseconds=10_000_000)
        later = now + delta
        command_msg.stamp = later.to_msg()

        self.command_publisher_.publish(command_msg)
        self.get_logger().info("已发布机器人控制命令")
    def twist_callback(self, msg):
        # 存储消息到缓冲区（这里使用简单的变量存储最新消息）
        self.cmd_buffer_ = msg
        self.new_msg_ = True
        
        # 获取旋转矩阵
        R_right = self.rpy_to_rotation_matrix(msg.right_arm_twist.angular.x, msg.right_arm_twist.angular.y, msg.right_arm_twist.angular.z)
        R_left = self.rpy_to_rotation_matrix(msg.left_arm_twist.angular.x, msg.left_arm_twist.angular.y, msg.left_arm_twist.angular.z)
        
        #R_right = self.rpy_to_rotation_matrix(0, 0, 0)
        #R_left = self.rpy_to_rotation_matrix(0, 0, 0)
        # 创建3x4的变换矩阵（旋转 + 平移）
        translation_right = np.array([msg.right_arm_twist.linear.x, msg.right_arm_twist.linear.y, msg.right_arm_twist.linear.z]) #+ self.current_pos[0] # 平移向量
        tf_3x4_right = np.hstack((R_right, translation_right.reshape(3, 1)))
        translation_left = np.array([msg.left_arm_twist.linear.x, msg.left_arm_twist.linear.y, msg.left_arm_twist.linear.z]) #+ self.current_pos[1] # 平移向量
        tf_3x4_left = np.hstack((R_left, translation_left.reshape(3, 1)))
        
        # 如果需要4x4齐次变换矩阵
        tf_4x4_right = np.vstack((tf_3x4_right, [0, 0, 0, 1]))
        tf_4x4_left = np.vstack((tf_3x4_left, [0, 0, 0, 1]))
        
        # # 打印接收到的消息信息
        # self.get_logger().info(
        #     f"arm_right received: linear.x={msg.right_arm_twist.linear.x:.3f} m/s, "
        #     f"linear.y={msg.right_arm_twist.linear.y:.3f} m/s, "
        #     f"linear.z={msg.right_arm_twist.linear.z:.3f} m/s, "
        #     f"angular.x={msg.right_arm_twist.angular.x:.3f} rad/s, "
        #     f"angular.y={msg.right_arm_twist.angular.y:.3f} rad/s, "
        #     f"angular.z={msg.right_arm_twist.angular.z:.3f} rad/s"
        #     f"arm_left received: linear.x={msg.left_arm_twist.linear.x:.3f} m/s, "
        #     f"linear.y={msg.left_arm_twist.linear.y:.3f} m/s, "
        #     f"linear.z={msg.left_arm_twist.linear.z:.3f} m/s, "
        #     f"angular.x={msg.left_arm_twist.angular.x:.3f} rad/s, "
        #     f"angular.y={msg.left_arm_twist.angular.y:.3f} rad/s, "
        #     f"angular.z={msg.left_arm_twist.angular.z:.3f} rad/s"
        # )

        # print(om.translation, om.rotation)
        # print(tf_4x4)
        dof, info = self.ik.ik(tf_4x4_right, tf_4x4_left, self.sorted_positions)
        # self.ik.debug_forward_kinematics()
        print("-----------dof------------",dof)
        if dof is not None:
            self.publish_command(dof)
        # self.ik.ik
def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()
    
    try:
        rclpy.spin(joint_state_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        joint_state_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
