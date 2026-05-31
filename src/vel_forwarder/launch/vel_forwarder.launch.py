import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    

    vel_forwarder = Node(
        package='vel_forwarder',
        executable='forwarder_node',
        name='vel_forwarder',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        remappings=[('ly/navi/vel', 'ly/control/vel')],
    )
    


    return LaunchDescription([
        vel_forwarder,
    ])