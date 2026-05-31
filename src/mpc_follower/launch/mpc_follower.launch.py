from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('mpc_follower')
    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    config_path = os.path.join(ws_root, 'src', 'mpc_follower', 'cfg', 'mpc_params.yaml')

    mpc_follower_node = Node(
        package='mpc_follower',
        executable='mpc_node',
        name='mpc_follower',
        output='screen',
        parameters=[config_path],
        arguments=['--ros-args', '--log-level', 'info'],
        # remappings=[('/cmd_vel', '/mpc_cmd_vel')],
    )

    return LaunchDescription([
        mpc_follower_node,
    ])