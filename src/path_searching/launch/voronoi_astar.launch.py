# Deprecated launch file. Use 'astar.launch.py' instead.
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('path_searching')
    config_path = os.path.join(pkg_share, 'cfg', 'plan_param.yaml')

    declare_param_file_cmd=DeclareLaunchArgument(
      'param_file',
      default_value= config_path
    )

    plan_manager = Node(
        package='path_searching',
        executable='plan_manager',
        name='plan_manager',
        output='screen',
        respawn=False,
        parameters=[LaunchConfiguration('param_file')],
    )

    return LaunchDescription([
        declare_param_file_cmd,
        plan_manager,
    ])