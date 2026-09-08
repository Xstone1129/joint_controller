from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("robot_lower_gateway"))
        / "config"
        / "erobot_v1.yaml"
    )
    config_file = LaunchConfiguration("config_file")
    enable_command_proxy = LaunchConfiguration("enable_command_proxy")
    command_proxy_prefix = LaunchConfiguration("command_proxy_prefix")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Robot-specific lower gateway YAML profile",
            ),
            DeclareLaunchArgument(
                "enable_command_proxy",
                default_value="false",
                description="Expose namespaced proxies to existing Ubuntu control services",
            ),
            DeclareLaunchArgument(
                "command_proxy_prefix",
                default_value="~/",
                description="Service prefix; '~/' resolves below /ubuntu_lower_gateway",
            ),
            Node(
                package="robot_lower_gateway",
                executable="robot_lower_gateway_node",
                name="ubuntu_lower_gateway",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "enable_command_proxy": ParameterValue(
                            enable_command_proxy, value_type=bool
                        ),
                        "command_proxy_prefix": command_proxy_prefix,
                    },
                ],
            ),
        ]
    )
