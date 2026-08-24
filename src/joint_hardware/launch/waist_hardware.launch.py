#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("joint_hardware")
    xacro_file = PathJoinSubstitution([package_share, "description", "waist_test.urdf.xacro"])
    controllers = PathJoinSubstitution([package_share, "config", "waist_controllers.yaml"])
    arguments = [
        DeclareLaunchArgument(
            "waist_port",
            default_value=(
                "/dev/serial/by-id/"
                "usb-STMicroelectronics_STM32_Virtual_ComPort_307A335D3435-if00"
            ),
        ),
        DeclareLaunchArgument("waist_node_id", default_value="1"),
        DeclareLaunchArgument("waist_channel", default_value="2"),
    ]
    robot_description = Command([
        FindExecutable(name="xacro"), " ", xacro_file,
        " waist_port:=", LaunchConfiguration("waist_port"),
        " waist_node_id:=", LaunchConfiguration("waist_node_id"),
        " waist_channel:=", LaunchConfiguration("waist_channel"),
    ])
    control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[{"robot_description": robot_description}, controllers],
    )
    state_broadcaster = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    waist_controller = Node(
        package="controller_manager", executable="spawner",
        arguments=["waist_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    start_state = RegisterEventHandler(OnProcessStart(
        target_action=control, on_start=[TimerAction(period=2.0, actions=[state_broadcaster])]
    ))
    start_waist = RegisterEventHandler(OnProcessExit(
        target_action=state_broadcaster, on_exit=[waist_controller]
    ))
    stop = RegisterEventHandler(OnProcessExit(
        target_action=control,
        on_exit=[EmitEvent(event=Shutdown(reason="ros2_control_node exited"))],
    ))
    return LaunchDescription(arguments + [control, start_state, start_waist, stop])
