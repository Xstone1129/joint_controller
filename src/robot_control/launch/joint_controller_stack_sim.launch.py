import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def package_launch_file(package_name, filename):
    package_share = get_package_share_directory(package_name)
    conventional_path = os.path.join(package_share, "launch", filename)
    if os.path.exists(conventional_path):
        return conventional_path
    return os.path.join(package_share, filename)


def generate_launch_description():
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_launch_file("erobot_controller", "load_controller_arm.launch.py")
        ),
        launch_arguments={
            "sim": "1.0",
            "use_rviz": "false",
            "enable_effort_mode_switch": "true",
            "allow_disabled_simulation_heavy_execution": "true",
        }.items(),
    )

    control_services_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_launch_file("robot_control", "robot_control.launch.py")
        )
    )

    lift_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_launch_file("joint_hardware", "lift_ethercat.launch.py")
        ),
        launch_arguments={
            "backend": "mock",
            "namespace": "lift",
            "brake_control_enabled": "true",
            "brake_release_wait_ms": "0",
            "use_persistent_zero_offset": "false",
        }.items(),
    )

    return LaunchDescription([controller_launch, lift_launch, control_services_launch])
