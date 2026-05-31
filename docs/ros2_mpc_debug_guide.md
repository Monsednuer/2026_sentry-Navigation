# ROS2 导航与 MPC 调试说明

这份说明按“能跑起来、能看效果、能定位问题”的顺序整理。下面命令默认在 Linux / WSL / 机器人主机上的 ROS2 工作区根目录执行，例如 `~/navigation_ws`。如果你的工作区名不同，把路径换成自己的即可。

## 1. 项目主链路

本项目不是单个节点，而是一条导航链：

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
path_searching/plan_manager  <--- /map, TF(map->base_link), 动态障碍/Costmap
        |
        v
/sPath, /ly/navi/reachable
        |
        +--> region_detector --> /ly/navi/directional_region, /ly/navi/should_rotate
        |                         |
        |                         v
        |                    speed_manager --> /setFollowSpeed
        |
        v
mpc_follower 或 path_following
        |
        v
/cmd_vel
        |
        v
vel_forwarder --> /ly/navi/vel 或 remap 后的 /ly/control/vel
```

注意：`mpc_follower`、`path_following/path_follower`、Nav2 `mppi_controller` 都会发布 `/cmd_vel`，同一时间只启动一种跟踪控制器，否则速度命令会互相打架。你现在要调 MPC，就优先只启动 `mpc_follower`。

## 2. 构建与环境

首次运行先装依赖并构建：

```bash
cd ~/navigation_ws
rosdep install --from-paths src -i -y
colcon build --symlink-install
source install/setup.bash
```

如果只改 MPC：

```bash
colcon build --packages-select mpc_follower --symlink-install
source install/setup.bash
```

MPC 包额外依赖 `nlopt`，如果构建时报 `nlopt not found`：

```bash
sudo apt install libnlopt-dev
```

每开一个新终端都要执行：

```bash
cd ~/navigation_ws
source install/setup.bash
```

## 3. 推荐的 MPC 全链路启动顺序

先保证定位/里程计已经在跑。MPC 至少需要：

- `/odom`: `nav_msgs/msg/Odometry`
- `/sPath`: `nav_msgs/msg/Path`
- `/setFollowSpeed`: `std_msgs/msg/Float64`，没有也能用默认速度，但建议启动速度管理
- 可选 `/dynamic_obstacles`: `sensor_msgs/msg/PointCloud2`

### 终端 1：启动地图和路径规划

推荐用 `navion.launch.py`，它会启动 `plan_manager`、`map_server`、生命周期管理器和静态 TF：

```bash
ros2 launch path_searching navion.launch.py
```

观察：

```bash
ros2 lifecycle get /map_server
ros2 topic echo /map --once
ros2 node info /plan_manager
```

如果想看 RViz：

```bash
rviz2 -d ~/navigation_ws/rviz/path_searching.rviz
```

### 终端 2：启动目标管理

正式参数：

```bash
ros2 launch navigoal_manager navigoal_pub.launch.py
```

仿真/测试参数：

```bash
ros2 launch navigoal_manager pubsimgoal.launch.py
```

手动发一个目标编号：

```bash
ros2 topic pub --once /ly/navi/goal std_msgs/msg/UInt8 "{data: 0}"
```

确认目标被转换为 `/goal_pose`：

```bash
ros2 topic echo /goal_pose --once
```

### 终端 3：启动区域检测和速度管理

`region_detector.launch.py` 会同时 include `speed_manager.launch.py`：

```bash
ros2 launch region_detector region_detector.launch.py
```

手动设置速度档位，合法值是 `0/1/2`：

```bash
ros2 topic pub --once /ly/navi/speed_level std_msgs/msg/UInt8 "{data: 1}"
ros2 topic echo /setFollowSpeed
```

### 终端 4：启动 MPC

```bash
ros2 launch mpc_follower mpc_follower.launch.py
```

观察 MPC 是否在输出速度：

```bash
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel
ros2 topic echo /ly/navi/reached
```

如果 `/cmd_vel` 一直是 0，优先检查：

```bash
ros2 topic hz /odom
ros2 topic echo /sPath --once
ros2 topic echo /setFollowSpeed --once
```

MPC 代码里如果发现 odom 没更新，会打印 `No new odom data!`，并主动发 0 速度。

### 终端 5：连接底盘时再启动速度转发

调 MPC 逻辑时可以先不启动它，避免真车运动。确认 `/cmd_vel` 合理后再启动：

```bash
ros2 launch vel_forwarder vel_forwarder.launch.py
```

该 launch 会把节点内部发布的 `/ly/navi/vel` remap 到 `/ly/control/vel`：

```bash
ros2 topic echo /ly/control/vel
```

## 4. 单独调试各模块

### 4.1 只调路径规划

只启动 `plan_manager`：

```bash
ros2 launch path_searching astar.launch.py
```

这个 launch 不会启动地图服务器，所以你必须另外提供 `/map`。更省心的方式是：

```bash
ros2 launch path_searching navion.launch.py
```

发目标后看是否有路径：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"

ros2 topic echo /sPath --once
ros2 topic echo /ly/navi/reachable --once
```

`plan_manager` 需要 TF 中存在 `map -> base_link` 或 `map -> baselink`，否则无法知道起点。

### 4.2 只调 MPC

启动：

```bash
ros2 launch mpc_follower mpc_follower.launch.py
```

MPC 的核心输入输出：

- 订阅 `/odom`
- 订阅 `/sPath`
- 订阅 `/setFollowSpeed`
- 订阅 `/dynamic_obstacles`
- 发布 `/cmd_vel`
- 发布 `/ly/navi/reached`

调参文件是：

```text
src/mpc_follower/cfg/mpc_params.yaml
```

常用调参入口：

- `Ts`: 控制周期，当前 0.05 s
- `N`: 预测步长，当前 30
- `MaxLinear/MinLinear`: 速度上下限
- `MaxAccelLinear/MinAccelLinear`: 加速度约束
- `Wx/Wy`: 跟踪路径位置误差权重
- `Wv`: 速度代价
- `target_speed`: 默认目标速度
- `reach_slow_distance`: 接近终点开始降速距离
- `reach_stop_distance`: 判定到达距离
- `ObstacleInflation/Wobs`: 动态障碍避障代价
- `GlobalMaxEval/LocalMaxEval`: NLOPT 优化迭代上限，卡顿时先降低，效果差时再提高

典型现象：

- `/sPath` 有数据、`/odom` 没数据：MPC 输出 0。
- `/odom` 时间戳不更新：MPC 输出 0，并报 `No new odom data!`。
- 接近终点后速度变小：这是 `reach_slow_distance` 的限速逻辑。
- 到终点后速度为 0：`/ly/navi/reached` 会发布 `true`。

### 4.3 只调区域限速

启动：

```bash
ros2 launch region_detector region_detector.launch.py
```

输入：

- `/sPath`
- `/ly/navi/speed_level`
- TF `map -> base_link`

输出：

- `/ly/navi/directional_region`
- `/ly/navi/should_rotate`
- `/setFollowSpeed`
- `visualization_marker`

区域配置：

```text
src/region_detector/cfg/region.yaml
src/speed_manager/cfg/speed.yaml
```

`speed.yaml` 里 `default` 当前是 `[1.5, 2.0, 2.5]`，所以速度档位 `1` 会输出 `2.0`。

### 4.4 只调旧版路径跟踪

旧版跟踪器：

```bash
ros2 launch path_following path_following.launch.py
```

它订阅 `/sPath` 和 `/setFollowSpeed`，发布 `/cmd_vel`。调 MPC 时不要同时启动它。

Nav2 MPPI 控制器：

```bash
ros2 launch path_following mppi_controller.launch.py
```

它会启动 Nav2 `controller_server`，并用 `path_to_follow_path_action.py` 把 `/sPath` 桥接成 `/mppi/follow_path` action。这个方案依赖 Nav2 controller、local costmap、`/odom` 和点云配置，适合和自研 MPC 做对比。

## 5. 每个功能包是干什么的

### mpc_follower

自研 MPC 路径跟踪控制器。用 `/odom` 作为当前状态，用 `/sPath` 作为参考路径，用 `/setFollowSpeed` 动态改目标速度，并输出 `/cmd_vel`。动态障碍来自 `/dynamic_obstacles`，内部用 PCL 聚类成圆形障碍，再放进 MPC 代价函数。

主要文件：

- `src/mpc_follower/src/mpc_controller.cpp`
- `src/mpc_follower/cfg/mpc_params.yaml`
- `src/mpc_follower/launch/mpc_follower.launch.py`

### path_searching

全局路径规划包。核心节点 `plan_manager` 订阅 `/map` 和 `/goal_pose`，通过 A*、ESDF、Voronoi/动态障碍处理和路径平滑生成 `/sPath`。也会发布 `/ly/navi/reachable` 告诉上层目标是否可达。

主要输入：

- `/map`
- `/goal_pose`
- `/cloud_livox_obs`
- `/costmap/costmap`
- `/costmap/costmap_updates`
- TF `map -> base_link` 或 `map -> baselink`

主要输出：

- `/sPath`
- `/ly/navi/reachable`

### navigoal_manager

目标编号到地图坐标的转换器。上层发 `/ly/navi/goal` 的 `UInt8` 编号后，它根据红蓝方目标表发布 `/goal_pose`。也支持 `/ly/navi/goal_pose` 的 `UInt16MultiArray` 坐标输入，并做一次固定坐标变换后发布 `/goal_pose`。

主要配置：

- `src/navigoal_manager/cfg/navigoal_param.yaml`
- `src/navigoal_manager/cfg/pubsimgoal.yaml`

### region_detector

区域和方向判断模块。它根据机器人当前 TF 位置、路径 `/sPath` 和 `region.yaml` 中的多边形区域，判断机器人是否正在某个需要特殊处理的区域内，并发布区域名和是否需要旋转。

输出会给 `speed_manager` 用，也能在 RViz 里看 `visualization_marker`。

### speed_manager

区域限速模块。它订阅 `region_detector` 的 `/ly/navi/directional_region` 和上层速度档位 `/ly/navi/speed_level`，查 `speed.yaml` 后发布 `/setFollowSpeed`。MPC 和旧版跟踪器都会吃这个速度。

### path_following

备用路径跟踪包，有两个路线：

- `path_follower`: 自研简单跟踪器，直接发布 `/cmd_vel`。
- `mppi_controller.launch.py`: 启动 Nav2 MPPI controller，并把 `/sPath` 桥接成 Nav2 `FollowPath` action。

它适合用来和 `mpc_follower` 对照，但不建议和 MPC 同时启动。

### vel_forwarder

底盘速度转发器。订阅 `/cmd_vel`，把 ROS 标准 `geometry_msgs/msg/Twist` 转成 `gimbal_driver/msg/Vel`。launch 中把 `ly/navi/vel` remap 到 `ly/control/vel`，用于对接下位机/云台驱动侧。

### stvl

STVL 调试脚本和配置目录，不是标准 ROS2 包。里面的 `force_stvl.sh` 会直接运行 Nav2 costmap，并加载 `spatio_temporal_voxel_layer` 插件。用于调动态障碍 costmap：

```bash
bash src/stvl/force_stvl.sh
bash src/stvl/kill_stvl.sh
```

注意脚本里有硬编码的 `~/navigation/src/stvl/...` 路径，如果你的工作区不是这个名字，需要先改路径。

### shell

临时启动脚本目录：

- `begin_serial.sh`: 启动 `mpc_follower` 和 `vel_forwarder`
- `start_map_server.sh`: 手动启动并激活 `nav2_map_server`

这些脚本也有硬编码路径，建议作为参考，不要盲跑。

## 6. 常用排查命令

看节点和话题：

```bash
ros2 node list
ros2 topic list
ros2 node info /mpc_follower
ros2 node info /plan_manager
```

看数据频率：

```bash
ros2 topic hz /odom
ros2 topic hz /sPath
ros2 topic hz /cmd_vel
```

看 TF：

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames
```

看地图服务：

```bash
ros2 lifecycle get /map_server
ros2 topic echo /map --once
```

看规划是否成功：

```bash
ros2 topic echo /goal_pose --once
ros2 topic echo /sPath --once
ros2 topic echo /ly/navi/reachable --once
```

看 MPC 是否有足够输入：

```bash
ros2 topic echo /odom --once
ros2 topic echo /sPath --once
ros2 topic echo /setFollowSpeed --once
ros2 topic echo /cmd_vel
```

## 7. 已知注意点

- `src/path_searching/launch/astar_rviz_map_server.launch.py` 当前引用了被注释掉的 `static_tf_pub`，直接运行可能报 `NameError`。优先用 `navion.launch.py` 或 `astar.launch.py`。
- `src/shell/*.sh` 和 `src/stvl/*.sh` 有硬编码路径，换工作区后需要修改。
- `README.md` 当前看起来有编码乱码，建议后续统一保存为 UTF-8。
- `mpc_follower` 的 launch 从源码目录读取 `src/mpc_follower/cfg/mpc_params.yaml`，所以改参数后通常重启 launch 即可，不一定需要重新构建。
- 真车调试时，先只看 `/cmd_vel`，确认方向和限幅合理后再启动 `vel_forwarder`。
