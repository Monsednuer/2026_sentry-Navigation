# 第二步实施流程：JPS 前端 + 时间分配接入

> 前置：第一步 ESDF 插值已完成。本步目标：JPS 替换 A* 做前端搜索，加梯形加减速时间分配，输出密采样 `/sPath`。**不接优化器**，A* 保留作 fallback（参数切换）。

## 涉及文件

```
src/path_searching/
├── include/jps.h          # [新]
├── src/jps.cpp            # [新]
├── src/plan_manager.cpp   # [改] 搜索段替换，参数切换
├── cfg/plan_param.yaml    # [改] 新增参数
└── CMakeLists.txt         # [改] jps.cpp 加入 path_searching_lib
```

---

## 流程 1：JPS 实现（jps.h/cpp，约 300 行）

### 接口设计

```cpp
namespace navi_planner {
class JPS {
public:
    void setEnvironment(ESDF_enviroment::Ptr env);   // 与 Astar 同样持有 esdf_1
    // 输入输出均为米制地图坐标；返回 false = 无路径
    bool search(const Eigen::Vector2d& start, const Eigen::Vector2d& goal,
                std::vector<Eigen::Vector2d>& waypoints);  // 拐点序列（含起终点）
};
}
```

### 核心要点

1. **邻居剪枝**（JPS 灵魂）：
   - 直线方向 (1,0)：自然邻居只有正前方 1 个；若侧方有障碍则产生强制邻居（斜前方）。
   - 对角方向 (1,1)：自然邻居 3 个（正前、两对角侧）；侧方障碍产生强制邻居。
2. **跳点递推 `jump(pos, dir)`**：沿 dir 一直走，遇到①目标点 ②有强制邻居的点 ③出界/障碍 停止；对角方向每步先向两正交分量各 jump 一次，命中则当前点也是跳点。
3. **开放列表**：二叉堆或 `std::priority_queue`，f = g + 欧氏启发。
4. **防切角**：对角移动 (dx,dy) 要求 (dx,0) 和 (0,dy) 两方向栅格都自由（用 `esdf_1->checkCollision` 判定）。
5. **碰撞判据**：与现有 A* 保持一致——checkCollision 用的是 bin_map（已含膨胀），不需要再加膨胀。
6. **路径还原**：父指针回溯 → 拐点序列（JPS 天然只含跳点=拐点，无需再做共线裁剪，但建议加一道共线点剔除保险）。
7. **坐标转换**：进出搜索时用现有 `Pos2index`/`Index2pos`，注意它是 row/col 序（index[0]=row=y, index[1]=col=x）。

### 参考

- DDR-opt（github.com/ZJU-FAST-Lab/DDR-opt）的 2D JPS 可直接裁剪；
- 或经典实现 jps_planner（KumarRobotics/jps3d）的 2D 退化版。

---

## 流程 2：时间分配（可放 jps.cpp 末尾或独立 time_allocator.h）

报告 5.5.3.2 思路一：

```cpp
// 输入：JPS 拐点 waypoints；输出：带时间戳的密采样点
struct TimedPoint { Eigen::Vector2d pos; double t; };
std::vector<TimedPoint> allocateTime(const std::vector<Eigen::Vector2d>& wps,
                                     double k1, double k2,
                                     double vmax, double amax, double dt);
```

步骤：
1. `s1` = 相邻拐点连线总长；`s2` = 每个中间拐点处转角（rad）总和。
2. 等效路程 `s = k1*s1 + k2*s2`（k2 初值 0.3，表征轮到底盘中心距离）。
3. 梯形加减速：若 s < vmax²/amax 则达不到 vmax（三角剖面），`T = 2*sqrt(s/amax)`；否则 `T = s/vmax + vmax/amax`。
4. 按 `dt`（0.1s）从 0 到 T 均匀取时刻 → 由梯形速度剖面算每时刻的等效弧长 → 映射回拐点折线上的位置（含转角处按 k2 折算减速）。
5. 输出密采样点。

> 注意：本步密采样点直接发 `/sPath`，t 只作为中间量（后续 MINCO 步才用）。为保证 MPC 跟踪平滑，建议同时输出**等弧长 5cm 重采样**版本发话题。

---

## 流程 3：plan_manager 接入

现有主流程（plan_manager.cpp）：
```
goal_callback(1566) → 搜索段(~1300-1450)：A* searchPath → 失败时 start/goal escape
                    → smoother_1.smooth(1458) → publishPath(1154) 发 /sPath
```

改动：

1. **新增成员与参数**：
```cpp
bool use_jps_frontend_;
double jps_time_k1_, jps_time_k2_, jps_vmax_, jps_amax_, jps_dt_, jps_sample_ds_;
navi_planner::JPS jps_planner_;
```
`map_callback` 里 `planner_1.setEnvironment(esdf_1)` 旁边加 `jps_planner_.setEnvironment(esdf_1);`

2. **搜索段分叉**（A* 调用处）：
```cpp
std::vector<Eigen::Vector2d> Path_2d;
bool success = false;
if (use_jps_frontend_) {
    success = jps_planner_.search(start_pos, goal_pos, Path_2d);
    if (!success && enable_start_escape_mode_ /* 复用现有逃生逻辑 */) { /* 原有 escape 不动 */ }
} else {
    // 原有 A* 调用，一行不动
}
```

3. **平滑段分叉**（1458 行附近）：
```cpp
if (use_jps_frontend_) {
    auto timed = allocateTime(Path_2d, jps_time_k1_, jps_time_k2_,
                              jps_vmax_, jps_amax_, jps_dt_);
    Path_2d = resampleByArcLength(timed, jps_sample_ds_);  // 5cm 等弧长
    // 跳过 smoother
} else {
    Path_2d = smoother_1.smooth(Path_2d, 0.3f, 0.04f);
}
```
后续 publishPath、`/ly/navi/reachable`、blocked-path 检查逻辑全部不动。

4. **重规划触发**（timer_callback 1731）不动——它只依赖发布的 `/sPath` 与 ESDF 检查，与前端无关。

---

## 流程 4：参数（plan_param.yaml 追加）

```yaml
    # ===================== [G] JPS 前端（第二步） =====================
    use_jps_frontend: false     # 先 false 灰度，验证后改 true
    jps_time_k1: 1.0
    jps_time_k2: 0.3
    jps_vmax: 1.8               # 与 mpc_follower MaxLinear 对齐
    jps_amax: 1.5
    jps_dt: 0.1
    jps_sample_ds: 0.05         # /sPath 等弧长采样间距
```

---

## 流程 5：验证（按顺序）

1. **离线单测**（可选，复用 tmp_esdf_test ziglang 环境）：随机障碍图 + 峡谷图跑 JPS，检查①路径无碰撞（沿折线 1cm 步长查 checkCollision）②防切角（斜线不擦障碍角）③与 A* 比长度（JPS ≤ A*×1.05）。
2. **RViz 对比**：`use_jps_frontend: false/true` 各跑一次同一 goal，对比 `/sPath` 形状——JPS 版应是折线+密采样点，无毛刺。
3. **MPC 链路**：起 mpc_follower 跟 JPS 路径，确认 `findNearestWaypoint` 正常、无震荡（折线拐角处 MPC 会切角，正常现象，第三步 MINCO 会修）。
4. **性能**：JPS 搜索耗时应比 A* 低一个量级（30m 全场 < 5ms）。

## 验收标准

- 同一 goal 下 JPS 路径长度 ≤ A* 路径 ×1.05，搜索耗时 < A* 的 1/5。
- `use_jps_frontend: false` 时行为与现状完全一致（回归零风险）。
- MPC 能完整跟踪到终点，`/ly/navi/reached` 正常触发。

## 已知边界（第三步才解决，本步不处理）

- 折线拐角无平滑 → MPC 跟踪拐角会减速/切角，属预期。
- 时间分配在折角处偏慢（思路一固有缺陷）→ 不影响本步（t 不外用），第三步思路二/两阶段优化解决。
