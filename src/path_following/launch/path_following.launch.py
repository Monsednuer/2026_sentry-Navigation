import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('path_following')
    config_path = os.path.join(pkg_share, 'cfg', 'follow_param.yaml')
    print("Using config file:", config_path)
    

    declare_param_file_cmd=DeclareLaunchArgument(
      'param_file',
      default_value= config_path
    )

    path_follower = Node(
        package='path_following',
        executable='path_follower',
        name='path_follower',
        output='screen',
        respawn=False,
        parameters=[LaunchConfiguration('param_file')],
    )

    return LaunchDescription([
        declare_param_file_cmd,
        path_follower,
    ])