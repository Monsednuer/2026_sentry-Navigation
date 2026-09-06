#ifndef REPLAN_FSM_HPP
#define REPLAN_FSM_HPP

// ===================== [MINCO_V3] 第五步：重规划状态机纯函数库（报告5.5.4.4）=====================
// header-only，无 ROS 依赖（仅 Eigen + minco），可本地桩编译测试。
// 语义契约见《第五步-重规划状态机与异步化接入流程.md》§2.2/§2.3。
// 本文件签名与语义已冻结：plan_manager（消费者）与本文件实现（含测试）可并行开发。

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "minco/minco.hpp"

namespace navi_planner
{

// ===== 模式选择（报告5.5.4.4 流程图"模式选择"节点）=====
enum class ReplanMode
{
    NONE,          // 无事可做（无 goal / 已到近终点）
    FULL,          // 完全重规划：JPS(当前位置→goal) + MINCO（报告"定位严重偏离轨迹/目标点大幅度跳变"）
    PARTIAL,       // 部分重规划：投影+回退保留前缀+拼接 JPS + MINCO（报告中间态：轨迹干涉）
    OPTIMIZE_ONLY  // 仅优化：旧轨迹等时间隔重采样 + MINCO，不经过 JPS（报告"轨迹无碰撞+目标不跳变+跟踪良好"）
};

struct ReplanInputs
{
    bool has_goal = false;           // 存在有效目标
    bool near_goal = false;          // 距 goal < stall_goal_distance_threshold(0.3m)，收工
    bool has_traj = false;           // 存在有效 last_minco_traj_
    bool goal_changed = false;       // goal_seq != 已规划进当前轨迹的 seq
    bool deviation_exceeded = false; // |机器人位姿 - 轨迹投影点| > traj_deviation_threshold(0.5m)
    bool traj_collision = false;     // 当前轨迹前视段(collision_check_ahead=2.0m) ESDF 侵入
    bool stall = false;              // 停滞（detectStall 判据，worker 侧计算）
    bool dirty = false;              // 合并动态障碍集自上次成功规划以来变化
    bool optimize_due = false;       // now - last_optimize_time > optimize_only_period(1.0s)
};

// 决策表（优先级自上而下，与规格 §2.2 一致）：
//   !has_goal || near_goal                              -> NONE
//   !has_traj || goal_changed || stall || deviation_exceeded -> FULL
//   traj_collision                                      -> PARTIAL
//   dirty || optimize_due                               -> OPTIMIZE_ONLY
//   其余                                                 -> NONE
ReplanMode selectReplanMode(const ReplanInputs &in);

// ===== 轨迹最近投影（报告："实时里程计位置在轨迹上的最近投影点"）=====

struct TrajProjection
{
    bool valid = false;        // 失败：空轨迹/非有限值
    double t = 0.0;            // 投影点轨迹时刻，∈[0, T_total]
    Eigen::Vector2d pos;       // 投影点位置 = traj.getPos(t)
    Eigen::Vector2d vel;       // 投影点速度 = traj.getVel(t)（供日志/调试）
};

// 实现：先 0.01s 粗采样全程取最优，再在最优时刻 ±0.01s 邻域黄金分割细化至 1e-4s。
// 要求：粗采样步数上限保护（如步长退化到 <1e-6 直接返回粗最优）；全程 O(1000) 次 getPos。
TrajProjection projectOnTrajectory(const minco::Trajectory<5> &traj, const Eigen::Vector2d &p);

// ===== 部分重规划种子（报告："往回找一段时间保留拼接"）=====

struct PartialSeed
{
    bool valid = false;
    double t_keep = 0.0;                   // = max(0, t_proj - lookback)
    Eigen::Matrix<double, 2, 3> head_pva;  // 旧轨迹在 t_keep 的 (pos, vel, acc) —— 新轨迹起点连续性锚点
    std::vector<Eigen::Vector2d> kept_pts; // (t_keep, t_proj) 开区间内按弧长 ~interval 采样的拼接前缀
                                           // （不含两端点；窗口过短时取中点 1 个；作为 JPS corners 的前缀）
    Eigen::Vector2d jps_start;             // 旧轨迹在 t_proj 的位置（JPS 搜索起点）
};

// 实现：head_pva 逐列取 traj 的 getPos/getVel/getAcc(t_keep)；
// kept_pts：对 [t_keep, t_proj] 时间窗先密采样（0.02s）算弧长，再按 interval(0.4m) 等弧长取中间点，
// 数量 0~3 量级；窗口弧长 < interval 时取时间中点 1 个；t_keep==t_proj 时 kept_pts 为空且仍 valid。
PartialSeed buildPartialSeed(const minco::Trajectory<5> &traj, double t_proj,
                             double lookback, double interval);

// ===== 仅优化种子（报告："从重映射位置开始通过相等时间间隔重新采样轨迹"，不经过 JPS）=====

struct OptimizeOnlySeed
{
    bool valid = false;
    Eigen::Matrix<double, 2, 3> head_pva;  // 旧轨迹在 t_start 的 PVA
    Eigen::Matrix2Xd inner_pts;            // (N-1) 个中间控制点，等时间隔采样旧轨迹
    Eigen::VectorXd durations;             // N 段，全部 = T_rem/N（严格均匀）
};

// 实现：T_rem = T_total - t_start；t_start >= T_total - 1e-3 时返回 invalid；
// N = max(2, round(剩余弧长 / interval))，剩余弧长由 0.02s 密采样累加；
// inner_pts.col(k) = traj.getPos(t_start + (k+1)*T_rem/N)，k=0..N-2；
// durations 全部 = T_rem/N；head_pva 取 t_start 处 PVA。
OptimizeOnlySeed buildOptimizeOnlySeed(const minco::Trajectory<5> &traj, double t_start,
                                       double interval);

// =====================================================================
// 实现（[MINCO_V3] 第五步，header-only；语义见上方注释与规格 §2.2/§2.3）
// =====================================================================

namespace impl_replan_fsm
{
// 内部工具：把 t 夹紧到 [0, T] 后取轨迹位置（getPos 对超界 t 会外推，必须先 clamp）
inline double clampTime(double t, double T)
{
    return std::min(T, std::max(0.0, t));
}

// 内部工具：黄金分割一维极小化 f(t)=||pos(t)-p||²，区间收敛至 tol（默认 1e-4s）
// 返回目标 t（已夹紧 [lo,hi]），minVal 输出对应最小距离平方。
inline double goldenMinimize(const minco::Trajectory<5> &traj, const Eigen::Vector2d &p,
                             double lo, double hi, double tol, double *minVal)
{
    if (!(hi - lo > 1e-12))
    {
        const Eigen::Vector2d pos = traj.getPos(0.5 * (lo + hi));
        if (minVal) *minVal = (pos - p).squaredNorm();
        return 0.5 * (lo + hi);
    }
    const double gr = 0.5 * (std::sqrt(5.0) - 1.0);  // 0.618
    double a = lo, b = hi;
    double tl = b - gr * (b - a);
    double tr = a + gr * (b - a);
    double fl = (traj.getPos(tl) - p).squaredNorm();
    double fr = (traj.getPos(tr) - p).squaredNorm();
    for (int it = 0; it < 80 && (b - a) > tol; it++)
    {
        if (fl > fr)
        {
            a = tl;
            tl = tr;
            fl = fr;
            tr = a + gr * (b - a);
            fr = (traj.getPos(tr) - p).squaredNorm();
        }
        else
        {
            b = tr;
            tr = tl;
            fr = fl;
            tl = b - gr * (b - a);
            fl = (traj.getPos(tl) - p).squaredNorm();
        }
    }
    // 收敛区间内取最优点再精算一次
    const double tBest = 0.5 * (a + b);
    const double c1 = 0.5 * (a + tl), c2 = 0.5 * (tr + b);
    double bt = tBest;
    double bv = (traj.getPos(tBest) - p).squaredNorm();
    const double v1 = (traj.getPos(c1) - p).squaredNorm();
    const double v2 = (traj.getPos(c2) - p).squaredNorm();
    if (v1 < bv) { bv = v1; bt = c1; }
    if (v2 < bv) { bv = v2; bt = c2; }
    if (minVal) *minVal = bv;
    return bt;
}
}  // namespace impl_replan_fsm

inline ReplanMode selectReplanMode(const ReplanInputs &in)
{
    // 决策表优先级自上而下（与规格 §2.2 一致，勿调整顺序）：
    if (!in.has_goal || in.near_goal) return ReplanMode::NONE;
    if (!in.has_traj || in.goal_changed || in.stall || in.deviation_exceeded) return ReplanMode::FULL;
    if (in.traj_collision) return ReplanMode::PARTIAL;
    if (in.dirty || in.optimize_due) return ReplanMode::OPTIMIZE_ONLY;
    return ReplanMode::NONE;
}

inline TrajProjection projectOnTrajectory(const minco::Trajectory<5> &traj, const Eigen::Vector2d &p)
{
    TrajProjection proj;
    if (traj.getPieceNum() == 0) return proj;          // 空轨迹
    if (!p.allFinite()) return proj;
    const double T = traj.getTotalDuration();
    if (!(T > 0.0) || !std::isfinite(T)) return proj;

    // ---- 阶段 1：0.01s 粗采样全程（步数 = ceil(T/0.01)+1，含两端）----
    const double dtc = 0.01;
    const int steps = static_cast<int>(std::ceil(T / dtc)) + 1;
    // 步长退化保护：T 极小（<1e-6）时直接退化到端点采样（满足头文件注释要求）
    if (T < 1e-6)
    {
        const Eigen::Vector2d p0 = traj.getPos(0.0);
        const Eigen::Vector2d p1 = traj.getPos(T);
        if (!p0.allFinite() || !p1.allFinite()) return proj;
        proj.t = ((p0 - p).squaredNorm() <= (p1 - p).squaredNorm()) ? 0.0 : T;
    }
    else
    {
        const int nSeg = steps - 1;
        int bestIdx = 0;
        double bestD2 = std::numeric_limits<double>::infinity();
        for (int k = 0; k < steps; k++)
        {
            const double t = T * k / nSeg;
            const Eigen::Vector2d pos = traj.getPos(t);
            if (!pos.allFinite()) return proj;
            const double d2 = (pos - p).squaredNorm();
            if (d2 < bestD2)
            {
                bestD2 = d2;
                bestIdx = k;
            }
        }
        const double tBest = T * bestIdx / nSeg;

        // ---- 阶段 2：最优时刻 ±0.01s 邻域黄金分割细化至 1e-4s ----
        const double lo = std::max(0.0, tBest - dtc);
        const double hi = std::min(T, tBest + dtc);
        double fineMin = std::numeric_limits<double>::infinity();
        double tFine = tBest;
        if (hi - lo > 1e-9)
        {
            tFine = impl_replan_fsm::goldenMinimize(traj, p, lo, hi, 1e-4, &fineMin);
        }
        // 取粗/细中更优者（防御边界邻域内函数不平滑）
        const double coarseD2 = (traj.getPos(tBest) - p).squaredNorm();
        if (std::isfinite(fineMin) && fineMin <= coarseD2)
        {
            proj.t = impl_replan_fsm::clampTime(tFine, T);
        }
        else
        {
            proj.t = tBest;
        }
    }

    proj.pos = traj.getPos(proj.t);
    proj.vel = traj.getVel(proj.t);
    if (!proj.pos.allFinite() || !proj.vel.allFinite()) return TrajProjection();  // 非有限 -> invalid
    proj.valid = true;
    return proj;
}

inline PartialSeed buildPartialSeed(const minco::Trajectory<5> &traj, double t_proj,
                                    double lookback, double interval)
{
    PartialSeed seed;
    if (traj.getPieceNum() == 0) return seed;
    if (!std::isfinite(t_proj) || !std::isfinite(lookback)) return seed;
    const double T = traj.getTotalDuration();
    if (!(T > 0.0) || !std::isfinite(T)) return seed;

    // t_proj 先 clamp 到 [0, T_total]（getPos 对超界 t 会外推，必须先 clamp）
    const double t_proj_c = impl_replan_fsm::clampTime(t_proj, T);
    const double t_keep = std::max(0.0, t_proj_c - lookback);
    seed.t_keep = t_keep;

    // head_pva 三列 = getPos/getVel/getAcc(t_keep)
    seed.head_pva.col(0) = traj.getPos(t_keep);
    seed.head_pva.col(1) = traj.getVel(t_keep);
    seed.head_pva.col(2) = traj.getAcc(t_keep);
    seed.jps_start = traj.getPos(t_proj_c);
    if (!seed.head_pva.allFinite() || !seed.jps_start.allFinite()) return seed;  // invalid

    // kept_pts：(t_keep, t_proj) 开区间按弧长 ~interval 取中间点
    const double dw = t_proj_c - t_keep;
    if (dw >= 1e-9)
    {
        // 0.02s 密采样算弧长表（M 段，含两端）
        const double dt_arc = 0.02;
        const int M = std::max(1, static_cast<int>(std::ceil(dw / dt_arc)));
        std::vector<double> cum(M + 1, 0.0);
        Eigen::Vector2d prev = traj.getPos(t_keep);
        for (int j = 1; j <= M; j++)
        {
            const Eigen::Vector2d cur = traj.getPos(t_keep + dw * j / M);
            cum[j] = cum[j - 1] + (cur - prev).norm();
            prev = cur;
        }
        const double s_w = cum[M];
        if (s_w < interval)
        {
            // 窗口弧长 < interval：取时间中点 1 点
            seed.kept_pts.push_back(traj.getPos(0.5 * (t_keep + t_proj_c)));
        }
        else
        {
            // 按 interval 等弧长取内部点（不含两端），数量 0~3
            int n = static_cast<int>(std::floor((s_w - 1e-9) / interval));
            n = std::min(3, std::max(1, n));
            for (int k = 1; k <= n; k++)
            {
                const double s_target = k * interval;  // 严格 < s_w（见上方减 eps 取整）
                int j = 1;
                while (j <= M && cum[j] < s_target) j++;
                if (j > M) j = M;
                const double seg = cum[j] - cum[j - 1];
                double frac = (seg > 1e-12) ? (s_target - cum[j - 1]) / seg : 0.0;
                frac = std::min(1.0, std::max(0.0, frac));
                const double tk = t_keep + dw * (j - 1 + frac) / M;
                seed.kept_pts.push_back(traj.getPos(tk));
            }
        }
        for (const Eigen::Vector2d &pt : seed.kept_pts)
        {
            if (!pt.allFinite()) return PartialSeed();  // invalid
        }
    }
    // t_keep == t_proj：kept_pts 为空但仍 valid
    seed.valid = true;
    return seed;
}

inline OptimizeOnlySeed buildOptimizeOnlySeed(const minco::Trajectory<5> &traj, double t_start,
                                              double interval)
{
    OptimizeOnlySeed seed;
    if (traj.getPieceNum() == 0) return seed;
    if (!std::isfinite(t_start)) return seed;
    const double T = traj.getTotalDuration();
    if (!(T > 0.0) || !std::isfinite(T)) return seed;

    if (t_start >= T - 1e-3) return seed;  // invalid：剩余不足 / 越界
    if (t_start < 0.0) t_start = 0.0;      // 防御：负起点夹到 0
    const double T_rem = T - t_start;

    // 剩余弧长：0.02s 密采样累加
    double s_rem = 0.0;
    {
        const double dt_arc = 0.02;
        const int M = std::max(1, static_cast<int>(std::ceil(T_rem / dt_arc)));
        Eigen::Vector2d prev = traj.getPos(t_start);
        for (int j = 1; j <= M; j++)
        {
            const Eigen::Vector2d cur = traj.getPos(t_start + T_rem * j / M);
            s_rem += (cur - prev).norm();
            prev = cur;
        }
    }
    if (!std::isfinite(s_rem)) return seed;

    const int N = std::max(2, static_cast<int>(std::lround(s_rem / std::max(1e-6, interval))));
    const double Tseg = T_rem / N;

    // inner_pts.col(k) = getPos(t_start + (k+1)*T_rem/N)，k=0..N-2
    seed.inner_pts.resize(2, N - 1);
    for (int k = 0; k < N - 1; k++)
    {
        const Eigen::Vector2d pt = traj.getPos(t_start + (k + 1) * Tseg);
        if (!pt.allFinite()) return seed;
        seed.inner_pts.col(k) = pt;
    }

    // durations 全部 = T_rem/N（严格均匀）
    seed.durations.resize(N);
    seed.durations.setConstant(Tseg);

    // head_pva = t_start 处 PVA
    seed.head_pva.col(0) = traj.getPos(t_start);
    seed.head_pva.col(1) = traj.getVel(t_start);
    seed.head_pva.col(2) = traj.getAcc(t_start);
    if (!seed.head_pva.allFinite()) return seed;

    seed.valid = true;
    return seed;
}

}  // namespace navi_planner

#endif  // REPLAN_FSM_HPP
