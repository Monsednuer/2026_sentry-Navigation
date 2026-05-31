import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    region_pkg_share = get_package_share_directory("region_detector")
    speed_pkg_share = get_package_share_directory("speed_manager")

    region_config = os.path.join(region_pkg_share, "cfg", "region.yaml")
    speed_launch = os.path.join(speed_pkg_share, "launch", "speed_manager.launch.py")

    region_detector = Node(
        package="region_detector",
        executable="region_detector_node",
        name="region_detector",
        output="screen",
        parameters=[{"config_file": region_config}],
    )

    speed_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(speed_launch),
    )

    return LaunchDescription([
        region_detector,
        speed_manager,
    ])
