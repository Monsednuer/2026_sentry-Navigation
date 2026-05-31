import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("speed_manager")
    config_path = os.path.join(pkg_share, "cfg", "speed.yaml")

    speed_manager = Node(
        package="speed_manager",
        executable="speed_manager_node",
        name="speed_manager",
        output="screen",
        parameters=[{"config_file": config_path}],
    )

    return LaunchDescription([
        speed_manager,
    ])
