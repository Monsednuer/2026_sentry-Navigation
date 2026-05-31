import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory("path_following")
    default_param_file = os.path.join(pkg_share, "cfg", "mppi_controller.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    param_file = LaunchConfiguration("param_file")
    path_topic = LaunchConfiguration("path_topic")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true.",
    )
    declare_param_file = DeclareLaunchArgument(
        "param_file",
        default_value=default_param_file,
        description="Full path to MPPI controller params file.",
    )
    declare_path_topic = DeclareLaunchArgument(
        "path_topic",
        default_value="/sPath",
        description="Global path topic bridged to FollowPath action.",
    )

    configured_params = RewrittenYaml(
        source_file=param_file,
        root_key="mppi",
        param_rewrites={"use_sim_time": use_sim_time},
        convert_types=True,
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        namespace="mppi",
        name="controller_server",
        output="screen",
        parameters=[configured_params],
        remappings=[
            ("odom", "/odom"),
            ("speed_limit", "/speed_limit"),
            ("cmd_vel", "/cmd_vel"),
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        namespace="mppi",
        name="lifecycle_manager_controller",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["controller_server"],
                "bond_timeout": 5.0,
            }
        ],
    )

    path_action_bridge = Node(
        package="path_following",
        executable="path_to_follow_path_action.py",
        name="path_to_follow_path_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "path_topic": path_topic,
                "action_name": "/mppi/follow_path",
                "controller_id": "FollowPath",
                "goal_checker_id": "general_goal_checker",
                "min_points": 2,
                "min_send_interval_sec": 0.3,
                "republish_if_same": False,
            }
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_param_file,
            declare_path_topic,
            controller_server,
            lifecycle_manager,
            path_action_bridge,
        ]
    )
