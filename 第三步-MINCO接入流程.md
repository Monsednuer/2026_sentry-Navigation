# 第三步实施流程：MINCO 轨迹接入 + 单阶段优化

> 前置：第二步 JPS+时间分配已跑通。本步目标：从 GCOPTER 裁剪 MINCO 轨迹库，2D 化改造，L-BFGS 单阶段优化跑通，输出光滑动力学可行轨迹发 `/sPath`。**只做单阶段（PRE 版梯度）**，L-BFGS 补丁与两阶段优化留到第四步。

## 涉及文件

```
src/path_searching/
├── include/
│   ├── minco/minco.hpp        # [新] 从 GCOPTER 裁剪，3D→2D
│   ├── minco/trajectory.hpp   # [新] 从 GCOPTER 裁剪（分段多项式+采样）
│   ├── minco/lbfgs.hpp        # [新] 从 GCOPTER 原样拷贝（补丁第四步再打）
│   └── traj_optimizer.h       # [新] 优化器封装
├── src/traj_optimizer.cpp     # [新]
├── src/plan_manager.cpp       # [改] 平滑段再加一层分叉
└── cfg/plan_param.yaml        # [改] 新增 [H] 段
```

---

## 流程 1：GCOPTER 裁剪（半天工作量）

仓库：github.com/ZJU-FAST-Lab/GCOPTER。**只取三个头文件**（全部 header-only，不需要编译 GCOPTER 本体）：

| GCOPTER 文件 | 处理 | 说明 |
|---|---|---|
| `gcopter/include/gcopter/minco.hpp` | **改 3D→2D** | MINCO 轨迹核心类 |
| `gcopter/include/gcopter/trajectory.hpp` | **改模板维度** | 分段多项式采样（getPos/getVel/getAcc/getJerk） |
| `gcopter/include/gcopter/lbfgs.hpp` | 原样拷贝 | L-BFGS 无约束优化器 |
| flatness.hpp / gcopter.hpp / voxel_map.hpp / sdlp.hpp / root_finder.hpp | **不要** | 无人机微分平坦、安全走廊等全部丢弃 |

### 2D 化改动清单（minco.hpp / trajectory.hpp）

1. 空间维度：MINCO_S3NU 中所有 `Eigen::Matrix<double, 3, ...>` 的空间维 3 改为 2（建议直接把类模板化成 `template<int D>`，实例化 D=2；或全局替换 3→2 后人工核对）。
2. 端点状态矩阵：headState/tailState 从 3×3（3维×位置/速度/加速度）变 2×3。
3. 删除所有与第三维（z 轴）相关的特化代码。
4. **yaw 不进轨迹**：全向底盘 yaw 由 MPC 沿切向自管理，轨迹只有 (x,y)。

### MINCO 核心数学（读源码时对照，避免迷路）

- N 段轨迹，段 i 是 2s-1 次多项式（s=3 时为 5 次），系数 c_i。
- 线性映射 `M(T)·c = b(q, v̂, â, T)`：给定端点 PVA 状态、中间控制点 q（N-1 个）、段时间 T，系数 c 由**带状线性方程组**唯一解出（O(N)）。
- 能量 `E = Σ∫‖d³p/dt³‖²dt` 是 (c,T) 的二次型，可解析求 ∂E/∂c、∂E/∂T。
- **伴随回传**：外部 cost 对采样点 (p,v,a) 求梯度后，经 ∂(p,v,a)/∂(c,t) 汇总成 ∂cost/∂c 和 ∂cost/∂T，再经 M 的转置解一次同样的带状方程组，得到 ∂cost/∂q、∂cost/∂T。MINCO_S3NU 的正向接口（setConditions/setParameters/getEnergy）和反向接口（梯度回传，名字类似 propagateGrad/getGrad——**以源码为准**）就是这两步。
- **时间无约束化**：`T_i = exp(τ_i)`，优化变量是 τ，dT/dτ = T。

### 验证裁剪正确性的最小单测

```cpp
// 给定 5 个控制点、均匀 T=1s、端点静止，构造 MINCO 轨迹：
// 1. 轨迹必须精确经过每个控制点（MINCO 特性）
// 2. 速度/加速度在段衔接处连续
// 3. getEnergy() > 0 且与手算（或扰动数值微分）梯度一致（容差 1e-4）
```
梯度数值微分校验是必做项，后面所有优化正确性都依赖它。

---

## 流程 2：优化器封装（traj_optimizer.h/cpp）

### 接口

```cpp
class TrajOptimizer {
public:
    void setEnvironment(ESDF_enviroment::Ptr env);
    // 输入：起点PVA/终点PVA + 中间控制点 + 初始段时间
    // 输出：优化后的 Trajectory（分段多项式，可直接采样）；false=优化失败
    bool optimize(const Eigen::Matrix<double,2,3>& headPVA,
                  const Eigen::Matrix<double,2,3>& tailPVA,
                  const Eigen::MatrixXd& initInnerPts,   // 2 × (N-1)
                  const Eigen::VectorXd& initT,          // N
                  Trajectory<5>& outTraj);
};
```

### Cost 函数（单阶段版 = 报告 PRE_OPTIMIZATION）

```
J = w_energy · E(MINCO能量)
  + Σ_采样点 [ w_coll · C(d(p))          // 避障
             + w_vel  · P(‖v‖, vmax)     // 速度越限
             + w_acc  · P(‖a‖, amax) ]   // 加速度越限
```

1. **采样**：每段按固定时间步（如 0.05s）采 (p, v, a)。
2. **避障项**：`d = getDistQuadratic(p)`，`grad = getGradQuadratic(p)`；
   `C(d) = (d_safe - d)³  当 d < d_safe，否则 0`（d_safe = esdf_grad_radius，三次方保证梯度连续）。
   对 p 的梯度 = -3(d_safe-d)² · grad，**直接用原始 grad，不做方向分解**（这就是 PRE 阶段，方向分解留第四步）。
3. **速度/加速度项**：`P(x, x_max) = (x² - x_max²)² 当 x > x_max`（报告原话"平方减阈值平方"），一次求导后只剩 4x(x²-x_max²) 量级，简单。
4. 全部梯度经 MINCO 伴随回传到 (q, τ)，交给 L-BFGS。

### L-BFGS 调用注意

- 变量打包：x = [q_x; q_y; τ]，维度 2(N-1)+N。
- 本步用 GCOPTER 原版 lbfgs.hpp，**容忍** `Linear Search Max` 假失败（返回码非 SUCCESS 但轨迹已可用）——收到该返回码时照样取结果、打印警告。第四步加补丁根治。
- 设 max_iterations=200，超时硬上限 opt_max_time_ms=80（wall clock 检查）。

---

## 流程 3：plan_manager 接入

在第二步的平滑段分叉上再加一层：

```cpp
if (use_jps_frontend_ && use_minco_backend_) {
    // 1. 从 JPS+时间分配的采样点构造控制点初值：
    //    每 opt_ctrl_pt_interval(0.4m) 取 1 个中间控制点 → N 段
    //    段时间初值 = 均匀（对应前端时间分配的 dt 累计）
    // 2. 端点 PVA：起点速度用当前里程计速度（重规划连续性），终点速度 0
    // 3. optimize() 成功 → Trajectory 按 5cm 弧长采样发 /sPath
    //    optimize() 失败 → fallback：退回第二步的 JPS 折线路径（打印警告，不发空）
} else if (use_jps_frontend_) {
    // 第二步逻辑，不动
} else {
    // 原 A*+smoother，不动
}
```

要点：
- **fallback 必须有**：优化不收敛时机器人不能没路径。
- 优化在**独立线程/回调组**跑，避免 80ms 长尾阻塞 timer_callback 的碰撞检查。
- 发布后把 `Trajectory` 对象缓存为成员，第四步的重规划状态机会复用（最近投影、轨迹拼接）。

---

## 流程 4：参数（plan_param.yaml 追加）

```yaml
    # ===================== [H] MINCO 后端（第三步） =====================
    use_minco_backend: false       # 灰度开关，验证后改 true
    opt_ctrl_pt_interval: 0.4      # 控制点平均间距（m）
    opt_sample_dt: 0.05            # cost 采样时间步（s）
    opt_max_iter: 200
    opt_max_time_ms: 80
    esdf_grad_radius: 0.6          # 避障梯度介入距离 d_safe（m）
    w_energy: 1.0
    w_collision: 500.0
    w_vel: 10.0
    w_acc: 10.0
    opt_vmax: 1.8                  # 与 jps_vmax / MPC MaxLinear 对齐
    opt_amax: 1.5
```

---

## 流程 5：验证（按顺序）

1. **MINCO 单测**：过控制点、连续性、梯度数值微分（流程 1 末尾）。
2. **静态优化对比**：同一 goal，`use_minco_backend` 关/开各一次，RViz 看轨迹——优化后应明显离开障碍物、无折角。
3. **动力学检查脚本**：沿输出轨迹按 10ms 采样，统计 max‖v‖、max‖a‖，应 ≤ opt_vmax/opt_amax ×1.05。
4. **耗时**：打印 optimize() wall time，30m 轨迹应 < 80ms（此时单阶段可能偏慢，第四步两阶段会降到 ~10ms）。
5. **MPC 链路**：全程跟踪，横向误差应优于第二步折线版；`/ly/navi/reached` 正常。
6. **fallback 演练**：故意把 esdf_grad_radius 调极大逼优化失败，确认回退 JPS 折线、机器人不停在原地。

## 验收标准

- 轨迹精确过控制点，v/a 连续，数值微分梯度校验通过。
- 优化后轨迹全程无碰撞（ESDF 距离 ≥ 机器人内切半径），v/a 限值内。
- 优化失败时 fallback 生效，系统不发空路径。
- `use_minco_backend: false` 时行为与第二步完全一致。

## 本步已知缺陷（第四步解决，先记录不处理）

- 狭窄隧道前后控制点被推离隧道 → 段时间畸变 → 速度不可用（加时间正则+两阶段）。
- 峡谷势谷处优化震荡、`Linear Search Max` 假失败（L-BFGS delta 退出补丁）。
- 30m 轨迹优化可能接近 80ms 上限（迭代轮次退出 + 放松收敛条件）。

---

## 实施记录（已完成部分）

### 落地文件

| 文件 | 说明 |
|---|---|
| `include/minco/lbfgs.hpp` | GCOPTER 原版 L-BFGS（Lewis-Overton 线搜索），原样拷贝 |
| `include/minco/trajectory.hpp` | `Piece<5>/Trajectory<5>` 2D 化，去掉 RootFinder |
| `include/minco/minco.hpp` | `MINCO_S3NU_2D`：带状系统 + O(N) LU + 能量解析式 + 伴随回传（GCOPTER 3D→2D） |
| `include/traj_optimizer.h` / `src/traj_optimizer.cpp` | 优化器封装 + `buildMincoInitialGuess`（折线→控制点/段时间初值）+ `sampleTrajectoryByArc` |
| `src/test_minco.cpp` | 单测：前向/梯度数值微分/优化/集成链路（CMake 目标 `test_minco`） |
| `src/plan_manager.cpp` | `[MINCO_V1]`：参数、`planMinco`、TF 差分速度估计、fallback |
| `cfg/plan_param.yaml` | `[H]` 参数块（`use_minco_backend: false` 灰度） |

### 与流程文档的偏差

1. **同步优化，未开独立线程**：plan_manager 是单执行器线程，costmap 回调会并发改写 ESDF，
   独立线程有竞态风险；本步靠 `opt_max_time_ms` 硬上限（进度回调里检查墙钟，超时取消迭代并取当前最优）。
   第四步做重规划状态机时一起异步化。
2. **新增安全护栏**：优化后代价若显著差于初值（>1.5×且绝对增量>1）判失败回退折线，
   防止 `Linear Search Max` 假失败时发布比初值更差的轨迹。
3. **段时间软屏障**：τ=ln T 加二次屏障（T∈[~0.05, ~30]s），防止带状方程组病态。
4. 起点速度不用里程计话题，用 plan_manager 已有 TF 位姿差分（与 mpc_follower 的 updateRobotState 同思路）。

### 本地验证结果（MinGW g++ 6.3 + Eigen 3.3.7，无 ROS 桩编译）

- 前向：轨迹精确过控制点、端点 PVA 满足、衔接 v/a/j 连续（<1e-8），
  能量解析式与 Simpson 数值积分相对误差 <1e-4。
- 梯度：全 cost（能量+避障+速度+加速度）对 (q, τ) 解析梯度 vs 中心差分，
  最大相对误差 2.5e-7（容差 1e-4）。
- 优化：直线初值压障碍物（d=0.18）→ 推至 0.60（d_safe），9.1ms。
- 集成（L 形拐点 14m，N=35 段）：初值贴障 0.51m → 0.59m，maxV=1.72≤1.8，maxA=1.38≤1.5，
  200 轮迭代 7.5ms（<80ms）。
- 已知现象：初值**直穿障碍中心**的病态场景仍会触发 `Linear Search Max` 假失败（第四步补丁）；
  生产环境 JPS 前端保证拐点不撞障，初值不会直穿障碍。

### 机器人主机待办验证（本机无 ROS）

1. `colcon build --packages-select path_searching`（新增 `test_minco` 可执行）。
2. 跑 `ros2 run path_searching test_minco` 复验单测。
3. 按流程 5 做 RViz 对比 / 动力学检查 / MPC 链路 / fallback 演练。

