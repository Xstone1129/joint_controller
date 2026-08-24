from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="1.0",
        description="仿真模式: 0.0=真机模式, 1.0=仿真模式",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="是否启动RViz: true=启动, false=不启动",
    )
    enable_effort_mode_switch_arg = DeclareLaunchArgument(
        "enable_effort_mode_switch",
        default_value="false",
        description="Allow the high-level service to switch into EFFORT mode",
    )
    allow_disabled_simulation_heavy_execution_arg = DeclareLaunchArgument(
        "allow_disabled_simulation_heavy_execution",
        default_value="false",
        description=(
            "Enable Heavy V1 mock-plant execution while simulated drives remain disabled"
        ),
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("erobot_controller"),
                    "robot_arm",
                    "robot_arm.urdf.xacro",
                ]
            ),
            " ",
            "sim:=",
            LaunchConfiguration("sim"),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controller_config_path = PathJoinSubstitution(
        [
            FindPackageShare("erobot_controller"),
            "robot_arm",
            "ros2_controllers_arm.yaml",
        ]
    )
    sim_controller_config_path = PathJoinSubstitution(
        [
            FindPackageShare("erobot_controller"),
            "robot_arm",
            "ros2_controllers_arm_sim.yaml",
        ]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="arm",
        output="screen",
        parameters=[robot_description],
    )

    hardware_interface_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="arm",
        parameters=[
            robot_description,
            controller_config_path,
        ],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster_arm"],
        namespace="arm",
        output="screen",
    )

    erobot_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["erobot_controller_arm"],
        namespace="arm",
        output="screen",
        condition=UnlessCondition(
            LaunchConfiguration("allow_disabled_simulation_heavy_execution")
        ),
    )

    sim_erobot_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["erobot_controller_arm", "--param-file", sim_controller_config_path],
        namespace="arm",
        output="screen",
        condition=IfCondition(
            LaunchConfiguration("allow_disabled_simulation_heavy_execution")
        ),
    )

    arm_control_mode_service_node = Node(
        package="robot_control_msg",
        executable="arm_control_mode_service",
        name="arm_control_mode_service",
        output="screen",
        parameters=[
            {
                "enable_effort_mode_switch": LaunchConfiguration(
                    "enable_effort_mode_switch"
                )
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace="arm",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("erobot_controller"), "robot_arm", "display_erobot.rviz"]
            ),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            sim_arg,
            use_rviz_arg,
            enable_effort_mode_switch_arg,
            allow_disabled_simulation_heavy_execution_arg,
            robot_state_pub_node,
            hardware_interface_node,
            rviz_node,
            joint_state_broadcaster,
            erobot_controller,
            sim_erobot_controller,
            arm_control_mode_service_node,
        ]
    )
