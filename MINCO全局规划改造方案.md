# 全局规划改造方案：A*+Smoother → JPS+MINCO（报告5.5）

> 目标：将 `path_searching` 的全局规划从「A* + 梯度平滑」替换为中科大哨兵2025技术报告 5.5 节的「JPS 前端 + MINCO 轨迹优化」方案；`mpc_follower`（MPC 控制器）**零改动**；其余模块（navigoal_manager / region_detector / speed_manager / vel_forwarder / path_following）**零改动**。

---

## 1. 现状与目标对照

| 环节 | 现状 | 报告5.5 目标 |
|---|---|---|
| 前端搜索 | A*（带障碍代价权重），`Astar.cpp` | **JPS**（无权重格子最快寻路） |
| 时间信息 | 无（纯几何路径） | 梯形加减速重采样：s = k1·s1 + k2·s2，均匀时间间隔 |
| 后端优化 | 梯度 smoother（`smoother.cpp`） | **MINCO 轨迹**（GCOPTER 裁剪移植）+ L-BFGS 无约束优化 |
| 障碍物梯度 | DynamicVoronoi ESDF，仅距离查询 | 同一 ESDF + **二次插值**平滑距离/梯度（解决梯度无效化） |
| 优化策略 | 单次平滑 | **两阶段优化**：PRE_OPTIMIZATION → FINELY_OPTIMIZATION |
| 重规划 | 阻塞截断 + block zone memory + 停滞触发 | **三策略**：全局重规划 / 仅优化 / 部分重规划（轨迹拼接） |
| 输出接口 | `/sPath`(nav_msgs/Path) + `/ly/navi/reachable` | **保持不变**（适配层密采样输出） |

### 接口约束（不动 MPC 的关键）

`mpc_follower` 消费 `/sPath` 的方式是**几何路径点**（`findNearestWaypoint` 按距离找最近点，`TargetSpeed` 参数定速）。因此：

- 新规划器只需把 MINCO 优化后的轨迹**按固定间距（建议 5cm）密采样**为 `nav_msgs/Path` 发布到 `/sPath`，MPC 无需任何修改。
- 额外收益：MINCO 轨迹天然光滑且动力学可行，MPC 跟踪质量会比现在（smoother 输出）明显提升。
- 可选增强：新增 `/sTraj` 话题发布带时间戳的轨迹点（位置+速度+时间），供以后升级 MPC 用，本期不做。

---

## 2. 新包结构

在 `src/path_searching` 内改造（保留包名，launch 与参数文件路径不变，降低迁移成本）：

```
src/path_searching/
├── include/
│   ├── esdf_map.h            # [改] 增加二次插值距离/梯度查询
│   ├── dynamicvoronoi.h      # [保留] brushfire ESDF 内核
│   ├── jps.h                 # [新] JPS 搜索
│   ├── minco_traj.h          # [新] MINCO 轨迹表示（时间分配+正逆变换）
│   ├── traj_optimizer.h      # [新] L-BFGS 两阶段优化器
│   └── lbfgs.hpp             # [新] L-BFGS 头文件库（含报告所述 delta 退出条件补丁）
├── src/
│   ├── jps.cpp
│   ├── minco_traj.cpp
│   ├── traj_optimizer.cpp
│   ├── esdf_map.cpp          # [改]
│   └── plan_manager.cpp      # [改] 重规划状态机重写，A*/smoother 调用摘除
└── cfg/plan_param.yaml       # [改] 新增参数段，旧 A*/smoother 参数废弃
```

删除：`Astar.h/cpp`、`smoother.h/cpp`、`cubic_spline/`（后端平滑被 MINCO 完全取代）。`benchmark_esdf.cpp` 保留用于 ESDF 性能回归。

---

## 3. 分模块实现要点

### 3.1 ESDF 改造（`esdf_map`）

现有 DynamicVoronoi brushfire 内核保留（0.1m 分辨率下全场重建 ~2ms，满足报告"每次点云重建"策略，无需 FIESTA 增量更新）。

新增两个查询接口，**这是整个后端优化的地基**：

```cpp
// 二次插值距离：按点相对 Cell 中心的位置，在两个方向的 2×3 方格中
// 用三个点拟合二次函数，得到平滑距离（替代双线性插值）
double getDistQuadratic(const Eigen::Vector2d& pos);
// 梯度直接取二次函数切线斜率（平滑，解决峡谷形障碍中间梯度无效化）
Eigen::Vector2d getGradQuadratic(const Eigen::Vector2d& pos);
```

另需 `pushOutOfObstacle(Eigen::Vector2d& pos)`：沿梯度方向迭代把占据点推到自由空间，供起点/终点排出使用（替代现有 start_escape / goal_escape 逻辑，可保留作为 fallback）。

### 3.2 JPS 前端（`jps.h/cpp`）

- 在膨胀后的二值栅格上做标准 2D JPS（跳点搜索）。膨胀半径 = 机器人内切圆半径 + 余量（复用 `robot_inscribed_radius` + `clearance_margin` 参数语义）。
- 输出：拐点序列（waypoints），不含时间。
- 参考实现：报告引用 FAST-Lab 的 DDR-opt（github.com/ZJU-FAST-Lab/DDR-opt）的 `jps_planner`，可直接裁剪其 2D JPS。

### 3.3 时间分配（梯形加减速重采样）

对 JPS 拐点序列：

1. 统计相邻点连线总长 `s1`、相邻三点转角总和 `s2`；等效路程 `s = k1·s1 + k2·s2`（k2 表征轮到底盘中心距离，初值 k1=1.0, k2≈0.3）。
2. 用 `amax`/`vmax` 梯形加减速算总时长 T。
3. 按期望时间间隔 dt（建议 0.1s）对轨迹重新采样，得到**均匀时间戳的路径点序列**。

### 3.4 MINCO 轨迹（`minco_traj.h/cpp`）

从 GCOPTER（github.com/ZJU-FAST-Lab/GCOPTER）裁剪：

- **只保留 2D**：三维微分平坦 → 二维全向微分平坦（大幅简化，yaw 不参与轨迹优化，由 MPC 沿切向处理）。
- 保留核心：`MINCO_S2NU`（MINCO 轨迹类，s=2 阶，即最小加速度 jerk 维度按需要取 3/4）、`minco::MINCO` 正逆变换（控制点+段时间 ↔ 多项式系数）、`Trajectory` 采样接口（`getPos(t)/getVel(t)/getAcc(t)`）。
- 删掉：安全走廊（SFC）、无人机模型、可视化等一切不需要的代码。

### 3.5 优化器（`traj_optimizer.cpp`，核心）

无约束优化变量 = 中间控制点 q + 段时间 T（经 τ 无约束变换），求解器 L-BFGS。

**L-BFGS 补丁**（报告 5.5.4.1，务必加）：

```cpp
// 在 param.past > 0 时，相对函数值下降足够小则判定收敛退出，
// 避免 "Linear Search Max" 假失败
if (param.past > 0 && fabs(finit - f) / (fabs(finit) + 1.0) < param.delta / param.past)
    return LBFGS_CONVERGENCE;
```
另加迭代轮次硬上限，控制单次优化 ≤ 80ms（实际两次优化各 <10ms 是可达的）。

**Cost 项**（全部在轨迹上离散采样后回传到 GradC/GradT）：

| 项 | 说明 |
|---|---|
| 避障 | 二次插值 ESDF 梯度，梯度介入半径可配；详见下方两阶段差异 |
| 速度 | (‖v‖² − vmax²)² 型二次 loss，超阈值才产生 |
| 加速度 | 同上，对 amax |
| 时间正则 | 段时间 / 平均段时间 约束在 [0.9, 1.1]，二次惩罚，仅第一阶段需要 |

**两阶段优化**（报告 5.5.4.2，本方案性能核心）：

1. **PRE_OPTIMIZATION**：梯度 = 二次插值原始梯度（不分解方向）。目标是把轨迹形状推出障碍物。配合时间正则防止控制点被推离狭窄区导致段时间畸变。
2. **FINELY_OPTIMIZATION**：梯度 = 原始梯度**减去沿轨迹速度方向的分量**（gradPos 只留法向）。再沿 gradPos 方向采样一个 step 后的 ESDF 值估算"可移动性"：若梯度模长接近 1（可自由移动）则 normalize 到 1；若 < 0.5（处于势谷，继续推会优化到另一端），用 `scale·√|gradPos|` 作为 violaPos 大小、gradPos 方向为梯度方向做特殊处理。目标是优化动力学特征（控制点等时间间隔分布）。

**初值**：JPS+时间分配后的采样点作为控制点初值，段时间取均匀 dt。

### 3.6 （可选，二期）前端时间分配增强——报告思路二

若思路一的重采样在折角处时间分配不合理导致后端扭麻花：在思路一结果上加一次**放松时间+碰撞条件的 MINCO 预优化**，再按思路一估总时长、均匀时间重采样 MINCO 轨迹，作为最终前端输出。建议一期先上思路一，出现"前端导致优化失败"再加。

### 3.7 重规划状态机（`plan_manager.cpp` 重写）

替换现有 timer_callback 全部触发逻辑，改为三策略（报告 5.5.4.4）：

```
触发检查（10Hz 定时器）
  ├─ 当前轨迹与障碍物干涉？（沿轨迹前向采样查 ESDF）
  │     ├─ 干涉段距离 < 阈值 → 部分重规划
  │     └─ 干涉严重/轨迹失效 → 全局重规划（JPS 重搜）
  ├─ 收到新 goal → 全局重规划
  ├─ 周期性（如 1Hz）→ 仅优化（控制点不动结构，重新跑两阶段优化）
  └─ 否则 → 不发新轨迹，MPC 继续跟旧轨迹
```

**部分重规划的轨迹拼接**（MINCO 特性，必做）：

1. 找里程计实时位置在当前轨迹上的最近投影点，**往回退 Δt（如 0.3s）**。
2. 保留该点之前的一小段轨迹（固定状态：位置+速度作为新起点约束）。
3. 从该点向后做 JPS → 优化，新旧轨迹在机器人当前位置基本无跳变。

**起终点处理**：起点/终点落在占据区时，先 `pushOutOfObstacle` 再规划（替代旧 escape 模式）；终点无法排出时发布 `/ly/navi/reachable = false`。

**输出适配**：优化完成后 `Trajectory` 按 5cm 弧长密采样 → `nav_msgs/Path` → `/sPath`（frame_id 用现有 `global_frame_`，即 map）。

---

## 4. 参数规划（`plan_param.yaml` 新增段）

```yaml
/**:
  ros__parameters:
    # ===== 前端 JPS =====
    jps_inflate_margin: 0.05        # 在 robot_inscribed_radius 基础上的额外膨胀

    # ===== 时间分配 =====
    time_k1: 1.0                    # 直线权重
    time_k2: 0.3                    # 转角权重（≈轮到底盘中心距离量级）
    time_vmax: 1.8                  # 与 MPC MaxLinear 对齐
    time_amax: 1.5
    time_dt: 0.1                    # 前端均匀时间间隔

    # ===== MINCO 优化 =====
    opt_ctrl_pt_interval: 0.4       # 控制点平均间距（m），两阶段优化可减少控制点数量
    opt_max_iter_pre: 200
    opt_max_iter_fine: 200
    opt_max_time_ms: 80
    esdf_grad_radius: 0.6           # ESDF 梯度介入优化的距离阈值
    w_collision: 500.0
    w_vel: 10.0
    w_acc: 10.0
    w_time_reg: 50.0                # 仅 PRE 阶段
    time_reg_lower: 0.9
    time_reg_upper: 1.1
    fine_grad_threshold: 0.5        # FINELY 阶段势谷判断阈值

    # ===== 重规划 =====
    replan_check_period: 0.1
    replan_full_min_interval: 0.5
    replan_optimize_only_period: 1.0
    partial_replan_lookback: 0.3    # 轨迹回退时间（s）
    collision_check_ahead: 2.0      # 前向检查距离（m）
```

废弃（不再读取）：`obstacle_cost_weight`、`dynamic_penalty_weight`、`blocked_zone_memory_*`、`enable_blocked_path_truncation`、`start_escape_*`、`goal_escape_*`（由 pushOutOfObstacle 取代）。地图/costmap 订阅相关参数保留。

---

## 5. 依赖清单（Linux 服务器）

| 依赖 | 用途 | 获取 |
|---|---|---|
| Eigen3 | 已有 | — |
| OpenCV | 已有（esdf_map 用） | — |
| GCOPTER | MINCO 裁剪来源 | github.com/ZJU-FAST-Lab/GCOPTER（只取 minco 相关头文件，不需要整个 repo 编译） |
| L-BFGS 实现 | 优化器 | GCOPTER 内自带 `lbfgs.hpp`（打 §3.5 补丁）；或 liblbfgs |
| JPS | 前端 | 参考 DDR-opt（github.com/ZJU-FAST-Lab/DDR-opt）2D JPS 裁剪，或自实现（~300 行） |

无新增系统级依赖，colcon 构建即可。C++ 标准 ≥ 14（GCOPTER 头文件用到）。

---

## 6. 实施步骤（建议顺序，每步可独立验证）

1. **ESDF 插值升级**：实现二次插值查询；在 `benchmark_esdf.cpp` 中对比双线性 vs 二次插值的距离/梯度平滑性（峡谷地形用例必测）。此时主链路不动。
2. **JPS 接入**：`plan_manager` 中 JPS 与 A* 并存（参数切换），输出拐点序列与 A* 路径对比验证。JPS 结果 + 时间分配 → 密采样发 `/sPath`，先**不接优化器**，用 MPC 跟"带时间戳的折线路径"跑通链路。
3. **MINCO 接入**：GCOPTER 裁剪 + 单阶段优化（只 PRE）跑通；RViz 对比优化前后轨迹。
4. **两阶段优化 + L-BFGS 补丁**：达到报告效果（两次优化各 <10ms，控制点等时分布）。
5. **重规划状态机**：三策略 + 轨迹拼接替换旧逻辑；动态障碍场景实测。
6. **联调**：navigoal_manager → 规划 → MPC 全链路；region_detector/speed_manager 验证（它们也消费 `/sPath`，接口未变，理论上无感）。

---

## 7. 风险与注意点

| 风险 | 应对 |
|---|---|
| ESDF 分辨率：现状 5cm，报告用 5cm/10cm 均可 | 保持现状；若 10cm 则确认狭窄隧道（哨兵过隧道场景）梯度仍有效 |
| 二次插值在地图边界/未观测区行为 | 边界外 clamp + 返回最大距离，梯度指向场内 |
| 部分重规划拼接处在高速下仍有跳变 | 回退时间 lookback 调大；拼接段标记为固定不可优化 |
| L-BFGS 无约束优化不保证硬避障 | 保持 MPC 端 costmap 避障项（`Wobs`）作为兜底，MPC 不动正好保留这层保护 |
| 优化耗时长尾（80ms 上限）阻塞控制 | 规划在独立回调组/线程跑，MPC 永远跟"最后一条有效轨迹" |
| yaw 不进优化 | 报告明确由控制器处理（切向/小陀螺切换），现有 MPC 自行管理 yaw，无冲突 |

## 8. 验收标准

- 静态全场：goal 30m，从收到 goal 到发布轨迹 ≤ 100ms；轨迹光滑无扭麻花；MPC 跟踪横向误差 < 现有方案。
- 狭窄隧道：可一次性规划穿过斜隧道（对比现状 A*+smoother 贴边/卡顿）。
- 动态障碍：10Hz 重规划检查下 CPU 占用不高于现状 +20%。
- 接口零回归：`/sPath`、`/ly/navi/reachable` 话题格式不变，MPC/path_following/region_detector 无修改编译通过。
