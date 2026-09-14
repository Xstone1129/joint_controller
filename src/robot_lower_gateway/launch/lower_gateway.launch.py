from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


DEFAULT_ENABLE_COMMAND_PROXY = False


def profile_enable_command_proxy(config_path: Path) -> bool:
    """Read the command-proxy switch from the deployment's robot profile.

    The workspace profile is the only configuration source for this gateway, so
    a deployment that needs the namespaced proxies does not depend on a
    root-owned systemd override.  ``ROBOT_GATEWAY_ENABLE_COMMAND_PROXY`` (used by
    ``run_gateway.sh``) and an explicit ``enable_command_proxy:=<bool>`` launch
    argument still take precedence over the profile value.
    """
    try:
        with config_path.open("r", encoding="utf-8") as stream:
            profile = yaml.safe_load(stream) or {}
    except OSError:
        return DEFAULT_ENABLE_COMMAND_PROXY
    parameters = (profile.get("ubuntu_lower_gateway") or {}).get("ros__parameters") or {}
    return bool(parameters.get("enable_command_proxy", DEFAULT_ENABLE_COMMAND_PROXY))


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("robot_lower_gateway"))
        / "config"
        / "erobot_v1.yaml"
    )
    enable_command_proxy = LaunchConfiguration("enable_command_proxy")
    command_proxy_prefix = LaunchConfiguration("command_proxy_prefix")
    lift_config = Path(get_package_share_directory("joint_hardware")) / "config" / "lift_hardware.yaml"
    with lift_config.open("r", encoding="utf-8") as stream:
        lift = yaml.safe_load(stream) or {}
    lift_parameters = {
        "heavy_v1": {
            "lift_enable_timeout_ms": int(lift["lift_enable_timeout_ms"]),
            "lift_min_position_m": float(lift["position_min_m"]),
            "lift_max_position_m": float(lift["position_max_m"]),
            "lift_position_tolerance_m": float(lift["lift_position_tolerance_m"]),
            "lift_max_velocity_mps": float(lift["max_velocity_mps"]),
            "lift_max_acceleration_mps2": float(lift["lift_max_acceleration_mps2"]),
            "allow_lift_only_follow": bool(lift["allow_lift_only_follow"]),
            "lift_only_arm_hold_position_tolerance_rad": float(
                lift["lift_only_arm_hold_position_tolerance_rad"]
            ),
            "lift_native_name": str(lift["lift_native_name"]),
            "heavy_lift_brake_native_service": str(
                lift["heavy_lift_brake_native_service"]
            ),
            "brake_lock_velocity_threshold_mps": float(
                lift["brake_lock_velocity_threshold_mps"]
            ),
        },
        "lift": {
            "canonical_joint_name": str(lift["lift_canonical_joint_name"]),
            "native_joint_name": str(lift["lift_native_joint_name"]),
        },
        "topics": {
            "lift_driver_status": str(lift["lift_driver_status_topic"]),
            "lift_control_status": str(lift["lift_control_status_topic"]),
            "lift_joint_state": str(lift["lift_joint_state_topic"]),
        },
        "native_services": {
            "lift_power": str(lift["lift_power_service"]),
            "lift_command": str(lift["lift_command_service"]),
            "lift_stop": str(lift["lift_stop_service"]),
            "lift_hold": str(lift["lift_hold_service"]),
            "lift_set_drive_zero": str(lift["lift_set_drive_zero_service"]),
        },
        "command_timeout_ms": {
            "lift_power": int(lift["lift_power_timeout_ms"]),
            "lift_command": int(lift["lift_command_timeout_ms"]),
            "lift_safety": int(lift["lift_safety_timeout_ms"]),
            "lift_set_drive_zero": int(lift["lift_set_drive_zero_timeout_ms"]),
        },
        "stale_timeout_ms": {
            "lift_status": int(lift["lift_status_stale_timeout_ms"]),
            "lift_joint_state": int(lift["lift_joint_state_stale_timeout_ms"]),
        },
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_command_proxy",
                default_value=str(
                    profile_enable_command_proxy(Path(default_config))
                ).lower(),
                description=(
                    "Expose namespaced proxies to existing Ubuntu control services; "
                    "defaults to ubuntu_lower_gateway.ros__parameters."
                    "enable_command_proxy in the robot profile"
                ),
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
                    default_config,
                    lift_parameters,
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
