# 2026_sentry-Navigation

ROS 2 sentry navigation workspace for map-based planning, path following, speed-region handling, and chassis velocity forwarding.

This repository is arranged as a ROS 2 workspace source tree. Clone it as the workspace root, build with `colcon`, then source the generated setup file before launching nodes.

## Project Overview

The current navigation pipeline is:

```text
/ly/navi/goal or /ly/navi/goal_pose
        |
        v
navigoal_manager
        |
        v
/goal_pose
        |
        v
path_searching / plan_manager
        |
        +--> /sPath
        +--> /ly/navi/reachable
        |
        v
path follower, MPC follower, or Nav2 MPPI controller
        |
        +--> /cmd_vel
        +--> /ly/navi/reached
        |
        v
vel_forwarder
        |
        v
/ly/control/vel
```

Optional speed-region logic runs alongside the planner:

```text
/sPath + /ly/navi/speed_level
        |
        v
region_detector
        |
        +--> /ly/navi/directional_region
        +--> /ly/navi/should_rotate
        |
        v
speed_manager
        |
        v
/setFollowSpeed
```

Only one controller should publish `/cmd_vel` at a time. Use either `mpc_follower`, `path_following/path_follower`, or the Nav2 MPPI launch, not several simultaneously.

## Packages

| Package | Purpose |
| --- | --- |
| `path_searching` | A* / Voronoi / ESDF based global planning. Publishes `/sPath` and `/ly/navi/reachable`. Includes map-server launch helpers and an ESDF benchmark executable. |
| `path_following` | Lightweight path follower and Nav2 MPPI bridge. Subscribes `/sPath` and `/setFollowSpeed`, publishes `/cmd_vel` and `/ly/navi/reached`. |
| `mpc_follower` | MPC-based path follower using NLOPT. Subscribes `/odom`, `/sPath`, `/setFollowSpeed`, optional `/dynamic_obstacles`; publishes `/cmd_vel` and `/ly/navi/reached`. |
| `navigoal_manager` | Converts navigation goal IDs or encoded goal poses into `/goal_pose`. |
| `region_detector` | Detects directional regions along the planned path and publishes region/rotation hints. |
| `speed_manager` | Maps directional region plus speed level to `/setFollowSpeed`. |
| `vel_forwarder` | Converts `/cmd_vel` into the chassis velocity message and remaps `/ly/navi/vel` to `/ly/control/vel` in its launch file. |

## Repository Layout

```text
.
├── docs/                         # Notes and debugging documents
├── rviz/                         # RViz config
├── script/                       # Map conversion and point-cloud helper scripts
└── src/
    ├── mpc_follower/
    ├── navigoal_manager/
    ├── path_following/
    ├── path_searching/
    ├── region_detector/
    ├── speed_manager/
    ├── stvl/
    └── vel_forwarder/
```

The repository contains map and point-cloud assets used by the current setup. Some `.pcd` files are larger than 50 MB, so GitHub may warn about them during push. Consider Git LFS if these assets continue to grow.

## Environment

Recommended target:

- Ubuntu 22.04
- ROS 2 Humble
- `colcon`
- `rosdep`
- Nav2 packages
- PCL, Eigen, TF2, visualization messages
- NLOPT for `mpc_follower`

Extra dependencies used by current packages:

- `libnlopt-dev`
- `libpcl-dev` / ROS PCL packages
- `nav2_map_server`
- `nav2_controller`
- `nav2_mppi_controller`
- `nav2_lifecycle_manager`
- `nav2_costmap_2d`
- `spatio_temporal_voxel_layer` if using the MPPI/STVL costmap configuration
- `gimbal_driver` if building `vel_forwarder` against the current chassis message type

Install common dependencies:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  libnlopt-dev \
  libpcl-dev
```

Then let `rosdep` install ROS package dependencies:

```bash
cd ~/navigation_ws
rosdep install --from-paths src -i -y
```

## Clone And Build

```bash
cd ~
git clone https://github.com/Monsednuer/2026_sentry-Navigation.git navigation_ws
cd navigation_ws
rosdep install --from-paths src -i -y
colcon build --symlink-install
source install/setup.bash
```

For a focused rebuild:

```bash
colcon build --packages-select path_searching --symlink-install
colcon build --packages-select mpc_follower --symlink-install
source install/setup.bash
```

Source the workspace in every new terminal:

```bash
cd ~/navigation_ws
source install/setup.bash
```

## Main Launch Flow

### 1. Start Planner And Map Server

`navion.launch.py` starts `plan_manager`, `nav2_map_server`, lifecycle management, and static TF publishers used by the current setup.

```bash
ros2 launch path_searching navion.launch.py
```

Useful checks:

```bash
ros2 lifecycle get /map_server
ros2 topic echo /map --once
ros2 node info /plan_manager
```

Open RViz manually when needed:

```bash
rviz2 -d ~/navigation_ws/rviz/path_searching.rviz
```

### 2. Start Goal Manager

```bash
ros2 launch navigoal_manager navigoal_pub.launch.py
```

Publish a goal ID:

```bash
ros2 topic pub --once /ly/navi/goal std_msgs/msg/UInt8 "{data: 0}"
```

Check the converted goal:

```bash
ros2 topic echo /goal_pose --once
```

### 3. Start Region And Speed Management

`region_detector.launch.py` also includes `speed_manager.launch.py`.

```bash
ros2 launch region_detector region_detector.launch.py
```

Set a speed level. Valid levels are `0`, `1`, and `2`.

```bash
ros2 topic pub --once /ly/navi/speed_level std_msgs/msg/UInt8 "{data: 1}"
ros2 topic echo /setFollowSpeed
```

### 4. Start One Path Follower

MPC follower:

```bash
ros2 launch mpc_follower mpc_follower.launch.py
```

Simple path follower:

```bash
ros2 launch path_following path_following.launch.py
```

Nav2 MPPI controller bridge:

```bash
ros2 launch path_following mppi_controller.launch.py
```

Check velocity output:

```bash
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel
ros2 topic echo /ly/navi/reached
```

### 5. Forward Velocity To Chassis

Start this only after `/cmd_vel` looks reasonable.

```bash
ros2 launch vel_forwarder vel_forwarder.launch.py
```

Check the remapped chassis velocity output:

```bash
ros2 topic echo /ly/control/vel
```

## Important Topics

| Topic | Type | Direction |
| --- | --- | --- |
| `/ly/navi/goal` | `std_msgs/msg/UInt8` | Goal ID input to `navigoal_manager` |
| `/ly/navi/goal_pose` | `std_msgs/msg/UInt16MultiArray` | Encoded goal-pose input to `navigoal_manager` |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Planner goal |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Static map input to planner |
| `/costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Optional dynamic costmap input |
| `/costmap/costmap_updates` | `map_msgs/msg/OccupancyGridUpdate` | Optional dynamic costmap update input |
| `/dynamic_obstacles` | `sensor_msgs/msg/PointCloud2` | Optional dynamic obstacle input |
| `/sPath` | `nav_msgs/msg/Path` | Planned path |
| `/ly/navi/reachable` | `std_msgs/msg/Bool` | Planner reachability result |
| `/ly/navi/speed_level` | `std_msgs/msg/UInt8` | Speed gear input, valid values 0/1/2 |
| `/ly/navi/directional_region` | `std_msgs/msg/String` | Region detector output |
| `/ly/navi/should_rotate` | `std_msgs/msg/Bool` | Region detector rotation hint |
| `/setFollowSpeed` | `std_msgs/msg/Float64` | Target speed for followers |
| `/odom` | `nav_msgs/msg/Odometry` | Odometry input for MPC |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Controller velocity output |
| `/ly/navi/reached` | `std_msgs/msg/Bool` | Goal reached flag |
| `/ly/control/vel` | chassis velocity message | Final remapped chassis velocity output |

## Configuration Files

| File | Notes |
| --- | --- |
| `src/path_searching/cfg/plan_param.yaml` | Planner, ESDF/Voronoi, dynamic obstacle, costmap, and replanning parameters. |
| `src/mpc_follower/cfg/mpc_params.yaml` | MPC horizon, weights, velocity/acceleration limits, obstacle cost, and stop/slowdown distances. |
| `src/path_following/cfg/follow_param.yaml` | Simple follower PID/lookahead/speed parameters. |
| `src/path_following/cfg/mppi_controller.yaml` | Nav2 MPPI controller and STVL local costmap parameters. |
| `src/navigoal_manager/cfg/navigoal_param.yaml` | Red/blue goal coordinate tables. |
| `src/region_detector/cfg/region.yaml` | Directional region polygons and direction pairs. |
| `src/speed_manager/cfg/speed.yaml` | Region-specific speed table for speed levels 0/1/2. |

## Debugging

Planner only:

```bash
ros2 launch path_searching astar.launch.py
```

Planner with map server:

```bash
ros2 launch path_searching navion.launch.py
```

Publish a manual goal:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

Check planner output:

```bash
ros2 topic echo /sPath --once
ros2 topic echo /ly/navi/reachable --once
```

MPC input checks:

```bash
ros2 topic hz /odom
ros2 topic echo /sPath --once
ros2 topic echo /setFollowSpeed --once
```

If MPC prints `No new odom data!`, verify that `/odom` is being published with updated timestamps.

Run the ESDF benchmark after building `path_searching`:

```bash
./install/path_searching/lib/path_searching/esdf_benchmark 200 250 0.05 10 5
```

## Notes For Linux Migration

- This repository includes `.gitattributes` to normalize source/config/script line endings to LF.
- Build artifacts such as `build/`, `install/`, and `log/` are ignored and should be regenerated on Linux.
- VS Code databases, Python cache files, and local Codex metadata are ignored.
- If shell scripts are not executable after cloning, run:

```bash
chmod +x src/shell/*.sh src/stvl/*.sh
```

## Known Caveats

- Several source comments and older docs appear to have been saved with a mismatched text encoding. The current README is UTF-8, but older markdown/config comments may still display incorrectly until those files are cleaned.
- `vel_forwarder` currently depends on `gimbal_driver` for the chassis `Vel` message even though local message definitions remain in `src/vel_forwarder/msg/`.
- The MPPI/STVL path needs the relevant Nav2 and STVL plugins installed in the target ROS 2 environment.
