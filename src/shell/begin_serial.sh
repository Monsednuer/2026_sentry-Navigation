#!/bin/bash

WS=~/navigation

# 在一个终端中启动所有节点
# gnome-terminal --title="SLAM Nodes" -- bash -c "
source $WS/install/setup.bash

# 启动 livox_ros_driver2
ros2 launch path_searching debug_astar.launch.py  &

# 启动 FAST-LIO2
ros2 launch mpc_follower mpc_follower.launch.py  &

# 启动 rotate_cloud_imu_node
ros2 run vel_forwarder forwarder_node  &


# 等待所有后台任务
wait
exec bash
"
