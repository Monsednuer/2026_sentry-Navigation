# #!/bin/bash
# pkill -f nav2_costmap_2d

# ros2 run nav2_costmap_2d nav2_costmap_2d --ros-args \
#   --params-file ~/navigation/src/stvl/config/stvl_simple.yaml \
#   -p plugins:='["stvl_layer"]' \
#   -p stvl_layer.plugin:='"spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"' \
#   -p stvl_layer.enabled:=true \
#   -p stvl_layer.voxel_decay:=5.0 \
#   -p stvl_layer.voxel_size:=0.2 \
#   -p stvl_layer.observation_sources:='"cloud"' \
#   -p stvl_layer.cloud.topic:='"/cloud_registered"' \
#   -p stvl_layer.cloud.data_type:='"PointCloud2"' \
#   -p stvl_layer.cloud.marking:=true \
#   -p stvl_layer.cloud.clearing:=true \
#   -p stvl_layer.cloud.model_type:=1 &

# sleep 2
# ros2 lifecycle set /costmap/costmap configure
# ros2 lifecycle set /costmap/costmap activate
#robot_base_frame:="base_link" \

pkill -f nav2_costmap_2d

ros2 run nav2_costmap_2d nav2_costmap_2d --ros-args \
  --params-file ~/navigation/src/stvl/config/stvl_simple.yaml \
  -p global_frame:="map" \
  -p robot_base_frame:="base_link" \
  -p rolling_window:=true \
  -p width:=20 \
  -p height:=10 \
  -p origin_x:=-2.0 \
  -p origin_y:=-5.0 \
  -p resolution:=0.1 \
  -p plugins:='["stvl_layer"]' \
  -p stvl_layer.plugin:='"spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"' \
  -p stvl_layer.enabled:=true \
  -p stvl_layer.voxel_size:=0.1 \
  -p stvl_layer.cloud.clearing:=false \
  -p stvl_layer.cloud.clearing_tolerance:=0.2 \
  -p stvl_layer.min_obstacle_height:=-0.5 \
  -p stvl_layer.max_obstacle_height:=3.0 \
  -p stvl_layer.cloud.voxel_min_points:=1 \
  -p stvl_layer.voxel_decay:=10.0 \
  -p stvl_layer.voxel_size:=0.1 \
  -p stvl_layer.observation_sources:='"cloud"' \
  -p stvl_layer.cloud.topic:='"/cloud_registered"' \
  -p stvl_layer.cloud.data_type:='"PointCloud2"' \
  -p stvl_layer.cloud.marking:=true \
  -p stvl_layer.cloud.clearing:=true \
  -p stvl_layer.cloud.model_type:=1 &

sleep 2
ros2 lifecycle set /costmap/costmap configure
ros2 lifecycle set /costmap/costmap activate
