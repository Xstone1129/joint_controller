#!/usr/bin/env python3

import atexit
from pathlib import Path
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def write_controller_parameters(lift):
    parameters = {
        "/**/lift_controller": {
            "ros__parameters": {
                "joint": str(lift["lift_canonical_joint_name"]),
                "position_min_m": float(lift["position_min_m"]),
                "position_max_m": float(lift["position_max_m"]),
                "max_velocity_mps": float(lift["max_velocity_mps"]),
                "max_acceleration_mps2": float(lift["max_acceleration_mps2"]),
                "max_jerk_mps3": float(lift["max_jerk_mps3"]),
                "default_velocity_scale": float(lift["default_velocity_scale"]),
                "jog_timeout_sec": float(lift["jog_timeout_sec"]),
                "brake_gate_stable_sec": float(lift["brake_gate_stable_sec"]),
                "driver_status_timeout_sec": float(lift["driver_status_timeout_sec"]),
                "target_stable_sec": float(lift["target_stable_sec"]),
                "goal_tolerance_m": float(lift["goal_tolerance_m"]),
                "stationary_velocity_mps": float(lift["stationary_velocity_mps"]),
                "status_rate_hz": float(lift["status_rate_hz"]),
            }
        }
    }
    output = tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", prefix="joint_lift_controller_", suffix=".yaml", delete=False
    )
    with output:
        yaml.safe_dump(parameters, output, sort_keys=False)
    path = Path(output.name)
    atexit.register(lambda: path.unlink(missing_ok=True))
    return str(path)


def generate_launch_description():
    config_path = Path(get_package_share_directory("joint_hardware")) / "config" / "lift_hardware.yaml"
    with config_path.open("r", encoding="utf-8") as stream:
        lift = yaml.safe_load(stream) or {}
    controller_parameters = write_controller_parameters(lift)

    def cfg(name, fallback):
        value = lift.get(name, fallback)
        if isinstance(value, bool):
            return str(value).lower()
        return str(value)

    package_share = FindPackageShare("joint_hardware")
    xacro_file = PathJoinSubstitution(
        [package_share, "description", "lift_test.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [package_share, "config", "lift_controllers.yaml"]
    )

    argument_specs = [
        ("backend", cfg("ethercat_backend", "mock"), "mock or etherlab"),
        ("namespace", "lift", "ROS namespace for the dedicated lift controller manager"),
        ("master_index", cfg("ethercat_master_index", 2), "EtherLab master index; 0/1 are reserved for the arms"),
        ("cycle_ms", cfg("ethercat_cycle_ms", 10), "EtherCAT cycle in milliseconds"),
        ("expected_working_counter", cfg("expected_working_counter", 1), "minimum complete PDO working counter"),
        ("slave_alias", cfg("slave_alias", 0), "EtherCAT slave alias"),
        ("slave_position", cfg("slave_position", 0), "absolute EtherCAT ring position"),
        ("slave_vendor_id", cfg("slave_vendor_id", 0), "vendor ID from ethercat slaves"),
        ("slave_product_code", cfg("slave_product_code", 0), "product code from ethercat slaves"),
        ("ethercat_interface", cfg("ethercat_interface", "enp5s0"), "EtherCAT interface name used by the master config"),
        ("dc_assign_activate", cfg("dc_assign_activate", 0), "AssignActivate from the LD3M ESI"),
        ("dc_sync0_shift_ns", cfg("dc_sync0_shift_ns", 0), "SYNC0 shift in nanoseconds"),
        ("command_units_per_rev", cfg("command_units_per_rev", 10000), "validated 6092/P00.08 command units per rev"),
        ("encoder_counts_per_rev", cfg("encoder_counts_per_rev", 131072), "validated 608F encoder counts per rev"),
        (
            "lead_mm_per_rev",
            cfg("lead_mm_per_rev", 3.333333333),
            "effective carriage travel in millimetres per motor revolution",
        ),
        ("lift_sign", cfg("lift_sign", -1.0), "mechanical direction sign"),
        ("position_min_m", cfg("position_min_m", -1.0), "software lower travel limit"),
        ("position_max_m", cfg("position_max_m", 0.0), "software upper travel limit"),
        ("max_rpm", cfg("max_rpm", 1080), "mechanical maximum motor speed"),
        (
            "position_limit_recovery_max_rpm",
            cfg("position_limit_recovery_max_rpm", 300),
            "maximum inward recovery speed while feedback is outside software travel",
        ),
        ("brake_control_enabled", cfg("brake_control_enabled", False), "allow explicit Operation enabled service requests"),
        ("limit_switch_enabled", cfg("limit_switch_enabled", False), "enable optional physical limit input"),
        ("limit_switch_positive_bit", cfg("limit_switch_positive_bit", -1), "60FD bit for the positive travel limit"),
        ("limit_switch_negative_bit", cfg("limit_switch_negative_bit", -1), "60FD bit for the negative travel limit"),
        ("limit_switch_active_high", cfg("limit_switch_active_high", True), "whether an asserted limit is a high bit"),
        ("brake_release_wait_ms", cfg("brake_release_wait_ms", -1), "confirmed P04.38 release delay"),
        ("brake_p04_37_ms", cfg("brake_p04_37_ms", 150), "drive P04.37 motor power-off delay"),
        ("brake_p04_39_rpm", cfg("brake_p04_39_rpm", 30), "drive P04.39 brake trigger speed"),
        ("brake_p06_14_ms", cfg("brake_p06_14_ms", 500), "drive P06.14 maximum stop time"),
        ("brake_p05_06_mode", cfg("brake_p05_06_mode", "drive_default"), "expected P05.06 disable mode"),
        ("brake_p05_10_mode", cfg("brake_p05_10_mode", "drive_default"), "expected P05.10 alarm-stop mode"),
        ("auto_fault_reset", cfg("auto_fault_reset", True), "bounded CiA 402 fault reset"),
        (
            "mode_mismatch_debounce_cycles",
            cfg("mode_mismatch_debounce_cycles", 5),
            "consecutive 10 ms 6061h mismatch cycles before latching a mode fault",
        ),
        (
            "use_persistent_zero_offset",
            cfg("use_persistent_zero_offset", True),
            "persist the current 6064h as the host-managed ROS coordinate zero",
        ),
        (
            "zero_offset_file",
            PathJoinSubstitution(
                [
                    EnvironmentVariable("HOME"),
                    ".local",
                    "state",
                    "joint_controller",
                    "lift_zero_offset.cfg",
                ]
            ),
            "host-side zero offset file",
        ),
    ]
    declared_arguments = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in argument_specs
    ]

    xacro_arguments = []
    for name, _, _ in argument_specs:
        if name == "namespace":
            continue
        xacro_arguments.extend([f" {name}:=", LaunchConfiguration(name)])
    robot_description = Command(
        [FindExecutable(name="xacro"), " ", xacro_file] + xacro_arguments
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{"robot_description": robot_description}, controllers_file, controller_parameters],
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            ["/", LaunchConfiguration("namespace"), "/controller_manager"],
        ],
        output="screen",
    )
    lift_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "lift_controller",
            "--controller-manager",
            ["/", LaunchConfiguration("namespace"), "/controller_manager"],
        ],
        output="screen",
    )

    start_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[joint_state_broadcaster],
                )
            ],
        )
    )
    start_lift_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[lift_controller],
        )
    )
    stop_on_control_failure = RegisterEventHandler(
        OnProcessExit(
            target_action=control_node,
            on_exit=[EmitEvent(event=Shutdown(reason="ros2_control_node exited"))],
        )
    )
    return LaunchDescription(
        declared_arguments + [
            control_node,
            start_controllers,
            start_lift_controller,
            stop_on_control_failure,
        ]
    )
