#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
MAP_YAML="${SRC_DIR}/path_searching/maps/map.yaml"

# 启动地图服务器
ros2 run nav2_map_server map_server \
  --ros-args \
  -p yaml_filename:="${MAP_YAML}" \
  -p frame_id:=map \
  -p topic_name:=map \
  -p use_sim_time:=false &

# 等待节点启动
sleep 2

# 激活生命周期
ros2 lifecycle set /map_server configure
sleep 1
ros2 lifecycle set /map_server activate

# 验证
echo "地图服务器状态："
ros2 lifecycle get /map_server
echo -e "\n当前发布的话题："
ros2 topic list | grep map

# 保持运行
wait
