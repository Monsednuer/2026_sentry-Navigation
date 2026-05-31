import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('path_searching')
    config_path = os.path.join(pkg_share, 'cfg', 'plan_param.yaml')
    print("Using config file:", config_path)

    ws_root = os.path.abspath(os.path.join(get_package_share_directory('path_searching'), '..', '..', '..', '..'))

    # 获取nav2包路径
    nav2_pkg_share = get_package_share_directory('nav2_map_server')

    # RViz配置路径  
    rviz_config_path = os.path.join(ws_root, 'rviz', 'path_searching.rviz')
    if not os.path.exists(rviz_config_path):
        print("RViz config file not found:", rviz_config_path)
    
    declare_param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=config_path
    )
    
    # 1. 先启动 plan_manager
    plan_manager = Node(
        package='path_searching',
        executable='plan_manager',
        name='plan_manager',
        output='screen',
        parameters=[LaunchConfiguration('param_file')],
        # 在plan_manager启动后添加延迟，确保其订阅者已建立
        arguments=['--ros-args', '--log-level', 'info'],
    )
    
    # 2. 启动RViz（延迟启动，让plan_manager先建立）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        # 依赖plan_manager节点
    )
    
    # 3. 最后启动地图服务器（延迟启动）
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': os.path.join(ws_root, 'src', 'path_searching', 'maps', 'map.yaml'),
            'topic_name': 'map',
            'frame_id': 'map',
            'use_sim_time': False,
        }],
        # 添加生命周期参数
        arguments=['--ros-args', '--log-level', 'info'],
    )
    
    # 4. 地图服务器生命周期管理节点
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'node_names': ['map_server'],
            'autostart': True,
            'bond_timeout': 5.0
        }],
        # 依赖map_server节点
    )
    
    # static_tf_pub = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_pub',
    #     output='screen',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
    # )
    
    # 使用TimerAction来延迟启动某些节点
    delayed_map_server = TimerAction(
        period=2.0,  # 延迟2秒启动地图服务器
        actions=[map_server]
    )
    
    delayed_lifecycle_manager = TimerAction(
        period=3.0,  # 延迟3秒启动生命周期管理器
        actions=[lifecycle_manager]
    )
    
    delayed_rviz = TimerAction(
        period=1.0,  # 延迟1秒启动RViz
        actions=[rviz_node]
    )

    return LaunchDescription([
        declare_param_file_cmd,
        # 先启动plan_manager
        plan_manager,
        static_tf_pub,
        # 然后启动RViz
        delayed_rviz,
        # 最后启动地图服务器
        delayed_map_server,
        delayed_lifecycle_manager,
    ])