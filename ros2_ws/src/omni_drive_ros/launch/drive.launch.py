"""Bring up the drive core with its parameter file."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_params = str(
        Path(get_package_share_directory("omni_drive_ros")) / "config" / "params.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="omni_drive_ros",
            executable="omni_drive_node",
            name="omni_drive_node",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=[
                ("cmd_vel", "/cmd_vel"),
                ("wheel_states", "/wheels/joint_states"),
                ("wheel_commands", "/wheels/commands"),
                ("imu/data", "/imu/data"),
                ("scan", "/scan"),
                ("estop", "/safety/estop"),
                ("odom", "/odom"),
            ],
        ),
    ])
