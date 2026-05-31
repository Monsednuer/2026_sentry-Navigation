# 2026_sentry-Navigation

这是一个面向哨兵机器人导航的 ROS 2 工作空间，当前主要包含全局路径规划、路径跟踪、区域限速、目标管理和底盘速度转发等模块。

本仓库本身就是一个 ROS 2 workspace 的根目录。迁移到 Linux 后，可以直接作为 `navigation_ws` 使用，通过 `colcon` 构建并运行各个 launch 文件。

## 项目总览

当前导航主链路如下：

```text
/ly/navi/goal 或 /ly/navi/goal_pose
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
路径跟踪器、MPC 跟踪器或 Nav2 MPPI 控制器
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

区域识别和限速模块会并行处理规划路径：

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

注意：同一时间只能启动一种会发布 `/cmd_vel` 的控制器。`mpc_follower`、`path_following/path_follower` 和 Nav2 MPPI 控制器不要同时运行，否则速度指令会互相覆盖。

## 功能包说明

| 功能包 | 作用 |
| --- | --- |
| `path_searching` | 全局路径规划模块，包含 A*、Voronoi、ESDF 相关逻辑；发布 `/sPath` 和 `/ly/navi/reachable`；包含地图服务 launch 和 ESDF benchmark。 |
| `path_following` | 简单路径跟踪器和 Nav2 MPPI 桥接模块；订阅 `/sPath`、`/setFollowSpeed`，发布 `/cmd_vel` 和 `/ly/navi/reached`。 |
| `mpc_follower` | 基于 NLOPT 的 MPC 路径跟踪器；订阅 `/odom`、`/sPath`、`/setFollowSpeed`，可选订阅 `/dynamic_obstacles`；发布 `/cmd_vel` 和 `/ly/navi/reached`。 |
| `navigoal_manager` | 将目标编号或编码后的目标位姿转换为 `/goal_pose`。 |
| `region_detector` | 根据规划路径识别方向区域，并发布区域名和是否需要旋转的提示。 |
| `speed_manager` | 根据方向区域和速度档位计算 `/setFollowSpeed`。 |
| `vel_forwarder` | 将 `/cmd_vel` 转换为底盘速度消息，并在 launch 中把 `/ly/navi/vel` remap 到 `/ly/control/vel`。 |

## 目录结构

```text
.
├── docs/                         # 调试说明和项目文档
├── rviz/                         # RViz 配置
├── script/                       # 地图转换、点云发布等辅助脚本
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

仓库中包含当前使用的地图和点云数据。部分 `.pcd` 文件超过 50 MB，GitHub 会给出大文件提醒；如果后续地图继续变大，建议改用 Git LFS 管理。

## 推荐环境

建议在以下环境中使用：

- Ubuntu 22.04
- ROS 2 Humble
- `colcon`
- `rosdep`
- Nav2 相关包
- PCL、Eigen、TF2、visualization messages
- `mpc_follower` 需要 NLOPT

当前项目额外依赖：

- `libnlopt-dev`
- `libpcl-dev` / ROS PCL 相关包
- `nav2_map_server`
- `nav2_controller`
- `nav2_mppi_controller`
- `nav2_lifecycle_manager`
- `nav2_costmap_2d`
- 使用 MPPI/STVL costmap 配置时需要 `spatio_temporal_voxel_layer`
- 构建 `vel_forwarder` 时需要当前底盘消息包 `gimbal_driver`

安装常用依赖：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  libnlopt-dev \
  libpcl-dev
```

然后使用 `rosdep` 安装 ROS 依赖：

```bash
cd ~/navigation_ws
rosdep install --from-paths src -i -y
```

## 克隆与构建

```bash
cd ~
git clone https://github.com/Monsednuer/2026_sentry-Navigation.git navigation_ws
cd navigation_ws
rosdep install --from-paths src -i -y
colcon build --symlink-install
source install/setup.bash
```

只重编某个功能包时：

```bash
colcon build --packages-select path_searching --symlink-install
colcon build --packages-select mpc_follower --symlink-install
source install/setup.bash
```

每次打开新终端后都需要 source 工作空间：

```bash
cd ~/navigation_ws
source install/setup.bash
```

## 主流程启动顺序

### 1. 启动规划器和地图服务

`navion.launch.py` 会启动 `plan_manager`、`nav2_map_server`、生命周期管理节点，以及当前配置中使用的静态 TF。

```bash
ros2 launch path_searching navion.launch.py
```

常用检查命令：

```bash
ros2 lifecycle get /map_server
ros2 topic echo /map --once
ros2 node info /plan_manager
```

需要查看 RViz 时手动启动：

```bash
rviz2 -d ~/navigation_ws/rviz/path_searching.rviz
```

### 2. 启动目标管理器

```bash
ros2 launch navigoal_manager navigoal_pub.launch.py
```

发布一个目标编号：

```bash
ros2 topic pub --once /ly/navi/goal std_msgs/msg/UInt8 "{data: 0}"
```

检查是否转换成 `/goal_pose`：

```bash
ros2 topic echo /goal_pose --once
```

### 3. 启动区域识别和速度管理

`region_detector.launch.py` 会同时 include `speed_manager.launch.py`。

```bash
ros2 launch region_detector region_detector.launch.py
```

设置速度档位。合法档位为 `0`、`1`、`2`：

```bash
ros2 topic pub --once /ly/navi/speed_level std_msgs/msg/UInt8 "{data: 1}"
ros2 topic echo /setFollowSpeed
```

### 4. 启动一种路径跟踪器

MPC 跟踪器：

```bash
ros2 launch mpc_follower mpc_follower.launch.py
```

简单路径跟踪器：

```bash
ros2 launch path_following path_following.launch.py
```

Nav2 MPPI 控制器桥接：

```bash
ros2 launch path_following mppi_controller.launch.py
```

检查速度输出：

```bash
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel
ros2 topic echo /ly/navi/reached
```

### 5. 将速度转发到底盘

确认 `/cmd_vel` 输出合理后，再启动底盘速度转发：

```bash
ros2 launch vel_forwarder vel_forwarder.launch.py
```

检查 remap 后的底盘速度输出：

```bash
ros2 topic echo /ly/control/vel
```

## 关键话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/ly/navi/goal` | `std_msgs/msg/UInt8` | 目标编号，输入给 `navigoal_manager`。 |
| `/ly/navi/goal_pose` | `std_msgs/msg/UInt16MultiArray` | 编码目标位姿，输入给 `navigoal_manager`。 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 规划器目标点。 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 静态地图，输入给规划器。 |
| `/costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 可选动态 costmap 输入。 |
| `/costmap/costmap_updates` | `map_msgs/msg/OccupancyGridUpdate` | 可选动态 costmap 增量更新。 |
| `/dynamic_obstacles` | `sensor_msgs/msg/PointCloud2` | 可选动态障碍物点云。 |
| `/sPath` | `nav_msgs/msg/Path` | 规划得到的路径。 |
| `/ly/navi/reachable` | `std_msgs/msg/Bool` | 规划是否可达。 |
| `/ly/navi/speed_level` | `std_msgs/msg/UInt8` | 速度档位输入，合法值为 0/1/2。 |
| `/ly/navi/directional_region` | `std_msgs/msg/String` | 当前路径所属方向区域。 |
| `/ly/navi/should_rotate` | `std_msgs/msg/Bool` | 是否建议旋转。 |
| `/setFollowSpeed` | `std_msgs/msg/Float64` | 跟踪器使用的目标速度。 |
| `/odom` | `nav_msgs/msg/Odometry` | MPC 使用的里程计输入。 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 控制器输出速度。 |
| `/ly/navi/reached` | `std_msgs/msg/Bool` | 是否到达目标点。 |
| `/ly/control/vel` | 底盘速度消息 | 最终发送到底盘控制侧的速度话题。 |

## 主要配置文件

| 文件 | 说明 |
| --- | --- |
| `src/path_searching/cfg/plan_param.yaml` | 规划器、ESDF/Voronoi、动态障碍物、costmap 和重规划参数。 |
| `src/mpc_follower/cfg/mpc_params.yaml` | MPC 预测步长、权重、速度/加速度约束、障碍物代价和到点减速参数。 |
| `src/path_following/cfg/follow_param.yaml` | 简单路径跟踪器的 PID、前视距离和目标速度参数。 |
| `src/path_following/cfg/mppi_controller.yaml` | Nav2 MPPI 控制器和 STVL local costmap 参数。 |
| `src/navigoal_manager/cfg/navigoal_param.yaml` | 红蓝方目标点坐标表。 |
| `src/region_detector/cfg/region.yaml` | 方向区域多边形和方向向量配置。 |
| `src/speed_manager/cfg/speed.yaml` | 不同区域在 0/1/2 档下的速度表。 |

## 调试命令

只启动规划器：

```bash
ros2 launch path_searching astar.launch.py
```

启动带地图服务的规划流程：

```bash
ros2 launch path_searching navion.launch.py
```

手动发布一个目标点：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

检查规划结果：

```bash
ros2 topic echo /sPath --once
ros2 topic echo /ly/navi/reachable --once
```

检查 MPC 输入：

```bash
ros2 topic hz /odom
ros2 topic echo /sPath --once
ros2 topic echo /setFollowSpeed --once
```

如果 MPC 输出日志中出现 `No new odom data!`，优先检查 `/odom` 是否在持续发布，并确认时间戳是否更新。

构建 `path_searching` 后可以运行 ESDF benchmark：

```bash
./install/path_searching/lib/path_searching/esdf_benchmark 200 250 0.05 10 5
```

## Linux 迁移注意事项

- 仓库已包含 `.gitattributes`，源码、配置和脚本会尽量保持 LF 换行。
- `build/`、`install/`、`log/` 不上传到 GitHub，迁移到 Linux 后重新构建即可。
- VS Code 数据库、Python 缓存和本地 Codex 元数据已被 `.gitignore` 排除。
- 如果 shell 脚本克隆后没有执行权限，可以运行：

```bash
chmod +x src/shell/*.sh src/stvl/*.sh
```

## 已知注意点

- 旧文档和部分源码注释可能存在编码不一致的问题，当前 README 已按 UTF-8 重新整理。
- `vel_forwarder` 当前使用的是 `gimbal_driver` 中的底盘 `Vel` 消息；虽然 `src/vel_forwarder/msg/` 中仍保留本地消息定义，但 CMake 中对应生成逻辑已注释。
- 使用 MPPI/STVL 路线时，需要目标系统中已安装 Nav2 和 STVL 相关插件。
