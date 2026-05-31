import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('navigoal_manager')

    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    config_path = os.path.join(ws_root, 'src', 'navigoal_manager', 'cfg', 'navigoal_param.yaml')

    declare_param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=config_path
    )
    
    goal_manager_node = Node(
        package='navigoal_manager',
        executable='goal_manager_node',
        name='goal_manager_node',
        parameters=[LaunchConfiguration('param_file')]
    )


    return LaunchDescription([
        declare_param_file_cmd,
        goal_manager_node,
    ])