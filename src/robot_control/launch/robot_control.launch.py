from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_path = LaunchConfiguration("urdf_path")
    ee_frame_left = LaunchConfiguration("ee_frame_left")
    ee_frame_right = LaunchConfiguration("ee_frame_right")
    joint_state_topic = LaunchConfiguration("joint_state_topic")
    motion_status_topic = LaunchConfiguration("motion_status_topic")

    common_parameters = {
        "urdf_path": urdf_path,
        "ee_frame_left": ee_frame_left,
        "ee_frame_right": ee_frame_right,
        "joint_state_topic": joint_state_topic,
        "motion_status_topic": motion_status_topic,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_arm_description"), "urdf", "right_left_arm.urdf"]
                ),
                description="Path to the arm-only URDF file",
            ),
            DeclareLaunchArgument(
                "ee_frame_left",
                default_value="lee_link",
                description="Left end-effector frame name",
            ),
            DeclareLaunchArgument(
                "ee_frame_right",
                default_value="ree_link",
                description="Right end-effector frame name",
            ),
            DeclareLaunchArgument(
                "joint_state_topic",
                default_value="/arm/joint_states",
                description="Joint state topic for arm-only control",
            ),
            DeclareLaunchArgument(
                "motion_status_topic",
                default_value="/arm/arm_controller/motion_status",
                description="Arm motion status topic",
            ),
            Node(
                package="robot_control",
                executable="cartesian_single_control_srv",
                name="cartesian_single_control_srv",
                parameters=[common_parameters],
                output="screen",
            ),
            Node(
                package="robot_control",
                executable="cartesian_path_absolute_control_srv",
                name="cartesian_path_absolute_control_srv",
                parameters=[common_parameters],
                output="screen",
            ),
            Node(
                package="robot_control",
                executable="cartesian_path_increment_control_srv",
                name="cartesian_path_increment_control_srv",
                parameters=[common_parameters],
                output="screen",
            ),
            Node(
                package="robot_control",
                executable="joint_absolute_control_srv",
                name="joint_absolute_control_srv",
                parameters=[common_parameters],
                output="screen",
            ),
            Node(
                package="robot_control",
                executable="joint_batch_control_srv",
                name="joint_batch_control_srv",
                parameters=[common_parameters],
                output="screen",
            ),
            Node(
                package="robot_control",
                executable="cartesian_moveL_path",
                name="cartesian_moveL_path",
                parameters=[common_parameters],
                output="screen",
            ),
        ]
    )
