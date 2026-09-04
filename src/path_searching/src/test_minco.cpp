// test_minco.cpp
// MINCO 后端单测（第三步验收第 1 项）：
//   1) 前向：轨迹精确过控制点、端点 PVA 满足、段衔接 v/a/j 连续、能量与数值积分一致
//   2) 梯度：全 cost（能量+避障+速度+加速度）解析梯度与数值微分一致（容差 1e-4）
//   3) 优化：完整 optimize() 收敛、轨迹推开障碍物、端点条件保持
// 构建（ROS 工作区）：colcon 中作为可执行目标；
// 本地验证：g++ -std=c++17 -I include -I <stub> -I <eigen> test_minco.cpp traj_optimizer.cpp
#include "traj_optimizer.h"
#include "minco/minco.hpp"

#include <Eigen/Core>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

// ===================== 解析距离场（单测专用，替代 ESDF） =====================
// d(p) = sqrt(eps^2 + ||p - o||^2)（光滑化的欧氏距离，模拟真实 ESDF：
// 梯度模长约等于 1 且处处不退化），grad = (p - o) / d
class FakeFieldOptimizer : public navi_planner::TrajOptimizer
{
public:
    static constexpr double kEps = 0.1;
    Eigen::Vector2d obstacle{2.5, 0.15};

protected:
    double queryDist(const Eigen::Vector2d &p) const override
    {
        return std::sqrt(kEps * kEps + (p - obstacle).squaredNorm());
    }
    Eigen::Vector2d queryGrad(const Eigen::Vector2d &p) const override
    {
        const double d = queryDist(p);
        return (p - obstacle) / d;
    }
};

// ===================== 测试 1：MINCO 前向 =====================
static bool test_forward()
{
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> upos(-5.0, 5.0);
    std::uniform_real_distribution<double> uvel(-1.0, 1.0);
    std::uniform_real_distribution<double> uacc(-0.5, 0.5);
    std::uniform_real_distribution<double> uT(0.3, 1.5);

    const int Ns[] = {1, 2, 3, 5, 8};
    for (int N : Ns)
    {
        Eigen::Matrix<double, 2, 3> headPVA, tailPVA;
        headPVA << upos(rng), uvel(rng), uacc(rng),
            upos(rng), uvel(rng), uacc(rng);
        tailPVA << upos(rng), uvel(rng), uacc(rng),
            upos(rng), uvel(rng), uacc(rng);

        Eigen::Matrix2Xd q(2, N - 1);
        for (int i = 0; i < N - 1; i++) q.col(i) = Eigen::Vector2d(upos(rng), upos(rng));
        Eigen::VectorXd T(N);
        for (int i = 0; i < N; i++) T(i) = uT(rng);

        minco::MINCO_S3NU_2D solver;
        solver.setConditions(headPVA, tailPVA, N);
        solver.setParameters(q, T);
        minco::Trajectory<5> traj;
        solver.getTrajectory(traj);

        // 端点 PVA
        const double eps = 1e-8;
        if ((traj.getJuncPos(0) - headPVA.col(0)).norm() > eps ||
            (traj.getJuncVel(0) - headPVA.col(1)).norm() > eps ||
            (traj.getJuncAcc(0) - headPVA.col(2)).norm() > eps ||
            (traj.getJuncPos(N) - tailPVA.col(0)).norm() > eps ||
            (traj.getJuncVel(N) - tailPVA.col(1)).norm() > eps ||
            (traj.getJuncAcc(N) - tailPVA.col(2)).norm() > eps)
        {
            std::printf("[FAIL] forward N=%d: boundary PVA mismatch\n", N);
            return false;
        }
        // 过控制点 + 衔接连续性
        for (int i = 1; i < N; i++)
        {
            if ((traj.getJuncPos(i) - q.col(i - 1)).norm() > eps)
            {
                std::printf("[FAIL] forward N=%d: junction %d not passing ctrl pt\n", N, i);
                return false;
            }
            const double durL = traj[i - 1].getDuration();
            const double dv = (traj[i - 1].getVel(durL) - traj[i].getVel(0.0)).norm();
            const double da = (traj[i - 1].getAcc(durL) - traj[i].getAcc(0.0)).norm();
            const double dj = (traj[i - 1].getJer(durL) - traj[i].getJer(0.0)).norm();
            if (dv > eps || da > eps || dj > eps)
            {
                std::printf("[FAIL] forward N=%d: discontinuity at junction %d (dv=%g da=%g dj=%g)\n",
                            N, i, dv, da, dj);
                return false;
            }
        }
        // 能量解析式与数值积分对照
        double energy = 0.0;
        solver.getEnergy(energy);
        double numE = 0.0;
        const int steps = 200;
        for (int i = 0; i < N; i++)
        {
            const double dur = traj[i].getDuration();
            const double dt = dur / steps;
            for (int k = 0; k < steps; k++)
            {
                // Simpson
                const double t0 = k * dt, t1 = (k + 0.5) * dt, t2 = (k + 1) * dt;
                numE += dt / 6.0 * (traj[i].getJer(t0).squaredNorm() +
                                    4.0 * traj[i].getJer(t1).squaredNorm() +
                                    traj[i].getJer(t2).squaredNorm());
            }
        }
        if (!(energy > 0.0) || std::fabs(energy - numE) > 1e-4 * std::max(1.0, numE))
        {
            std::printf("[FAIL] forward N=%d: energy %.10f vs numeric %.10f\n", N, energy, numE);
            return false;
        }
    }
    std::printf("[PASS] forward: pass-through / continuity / energy\n");
    return true;
}

// ===================== 测试 2：梯度数值微分校验 =====================
static bool test_gradient()
{
    FakeFieldOptimizer opt;
    navi_planner::TrajOptParam p;
    p.sample_dt = 0.05;
    p.esdf_grad_radius = 0.6;
    p.w_energy = 1.0;
    p.w_collision = 500.0;
    p.w_vel = 10.0;
    p.w_acc = 10.0;
    p.vmax = 0.8;  // 故意收紧，让速度惩罚生效
    p.amax = 1.2;
    opt.setParam(p);

    std::mt19937 rng(7);
    std::uniform_real_distribution<double> ujit(-0.4, 0.4);
    std::uniform_real_distribution<double> uT(0.3, 0.8);

    double maxRelErr = 0.0;
    for (int N = 2; N <= 6; N++)
    {
        Eigen::Matrix<double, 2, 3> headPVA, tailPVA;
        headPVA << 0.0, 0.3 + ujit(rng), 0.0,
            0.0, ujit(rng) * 0.3, 0.0;
        tailPVA << 1.5 * N, 0.0, 0.0,
            ujit(rng), 0.0, 0.0;

        // 沿直线撒控制点并加扰动，障碍物放在路径中部附近触发避障代价
        Eigen::Matrix2Xd q(2, N - 1);
        for (int i = 0; i < N - 1; i++)
        {
            q.col(i) = Eigen::Vector2d(1.5 * (i + 1) + ujit(rng), ujit(rng));
        }
        opt.obstacle = Eigen::Vector2d(0.75 * N + ujit(rng) * 0.3, ujit(rng) * 0.2);

        Eigen::VectorXd T(N);
        for (int i = 0; i < N; i++) T(i) = uT(rng);

        opt.prepare(headPVA, tailPVA, N);
        Eigen::VectorXd x(3 * N - 2);
        x.segment(0, N - 1) = q.row(0).transpose();
        x.segment(N - 1, N - 1) = q.row(1).transpose();
        for (int i = 0; i < N; i++) x(2 * (N - 1) + i) = std::log(T(i));

        Eigen::VectorXd g;
        const double f0 = opt.evaluateCost(x, g);
        if (!std::isfinite(f0))
        {
            std::printf("[FAIL] gradient N=%d: non-finite cost\n", N);
            return false;
        }

        double caseMax = 0.0;
        for (int j = 0; j < x.size(); j++)
        {
            const double h = 1e-6 * std::max(1.0, std::fabs(x(j)));
            Eigen::VectorXd xp = x, xm = x, gp, gm;
            xp(j) += h;
            xm(j) -= h;
            const double fp = opt.evaluateCost(xp, gp);
            const double fm = opt.evaluateCost(xm, gm);
            const double num = (fp - fm) / (2.0 * h);
            const double err = std::fabs(g(j) - num) / std::max(1.0, std::fabs(num));
            caseMax = std::max(caseMax, err);
            if (err > 1e-4)
            {
                std::printf("[FAIL] gradient N=%d var %d: analytic=%.10f numeric=%.10f relErr=%.3e\n",
                            N, j, g(j), num, err);
                return false;
            }
        }
        maxRelErr = std::max(maxRelErr, caseMax);
    }
    std::printf("[PASS] gradient: max relative error vs central diff = %.3e (< 1e-4)\n", maxRelErr);
    return true;
}

// ===================== 测试 3：完整优化 =====================
static bool test_optimize()
{
    FakeFieldOptimizer opt;
    navi_planner::TrajOptParam p;
    p.sample_dt = 0.05;
    p.max_iter = 200;
    p.max_time_ms = 2000.0;
    p.esdf_grad_radius = 0.6;
    p.w_energy = 1.0;
    p.w_collision = 500.0;
    p.w_vel = 1.0;
    p.w_acc = 1.0;
    p.vmax = 1.8;
    p.amax = 1.5;
    opt.setParam(p);

    const int N = 6;
    const double L = 5.0;
    Eigen::Matrix<double, 2, 3> headPVA, tailPVA;
    headPVA << 0.0, 0.5, 0.0,
        0.0, 0.0, 0.0;
    tailPVA << L, 0.0, 0.0,
        0.0, 0.0, 0.0;

    Eigen::Matrix2Xd q(2, N - 1);
    Eigen::VectorXd T(N);
    for (int i = 0; i < N - 1; i++) q.col(i) = Eigen::Vector2d(L * (i + 1) / N, 0.0);
    T.setConstant(L / N / 1.0);  // 约 1 m/s

    // 障碍物压在直线初值上：优化前最小距离很小，优化后应显著增大
    opt.obstacle = Eigen::Vector2d(2.5, 0.15);
    double initMinDist = 1e9;
    for (int k = 0; k <= 100; k++)
    {
        const Eigen::Vector2d pos(L * k / 100.0, 0.0);
        initMinDist = std::min(initMinDist, std::sqrt(FakeFieldOptimizer::kEps * FakeFieldOptimizer::kEps + (pos - opt.obstacle).squaredNorm()));
    }

    minco::Trajectory<5> traj;
    const bool ok = opt.optimize(headPVA, tailPVA, q, T, traj);
    if (!ok)
    {
        std::printf("[FAIL] optimize: returned false\n");
        return false;
    }

    double minDist = 1e9, maxV = 0.0, maxA = 0.0;
    const int steps = 400;
    const double total = traj.getTotalDuration();
    for (int k = 0; k <= steps; k++)
    {
        const double t = total * k / steps;
        const Eigen::Vector2d pos = traj.getPos(t);
        minDist = std::min(minDist, std::sqrt(FakeFieldOptimizer::kEps * FakeFieldOptimizer::kEps + (pos - opt.obstacle).squaredNorm()));
        maxV = std::max(maxV, traj.getVel(t).norm());
        maxA = std::max(maxA, traj.getAcc(t).norm());
    }
    const double endErr = (traj.getPos(total) - Eigen::Vector2d(L, 0.0)).norm();
    const double startErr = (traj.getPos(0.0) - Eigen::Vector2d(0.0, 0.0)).norm();
    const double endVel = traj.getVel(total).norm();

    std::printf("  optimize: cost=%.4f iters=%d time=%.1fms ret=%d\n",
                opt.lastCost(), opt.lastIterations(), opt.lastDurationMs(), opt.lastReturnCode());
    std::printf("  optimize: minDist %.4f -> %.4f, maxV=%.3f maxA=%.3f, startErr=%.2e endErr=%.2e endVel=%.3f\n",
                initMinDist, minDist, maxV, maxA, startErr, endErr, endVel);

    if (!(minDist > initMinDist))
    {
        std::printf("[FAIL] optimize: trajectory did not move away from obstacle\n");
        return false;
    }
    if (startErr > 1e-6 || endErr > 1e-6 || endVel > 1e-6)
    {
        std::printf("[FAIL] optimize: boundary conditions broken\n");
        return false;
    }
    std::printf("[PASS] optimize: collision pushed away, boundaries preserved\n");
    return true;
}

// ===================== 测试 4：集成链路（模拟 plan_manager planMinco） =====================
static bool test_integration()
{
    FakeFieldOptimizer opt;
    navi_planner::TrajOptParam p;
    p.sample_dt = 0.05;
    p.max_iter = 200;
    p.max_time_ms = 80.0;  // 与线上一致的硬上限
    p.esdf_grad_radius = 0.6;
    p.w_energy = 1.0;
    p.w_collision = 500.0;
    p.w_vel = 10.0;
    p.w_acc = 10.0;
    p.vmax = 1.8;
    p.amax = 1.5;
    opt.setParam(p);

    // L 形 JPS 拐点（含起终点）：折线本身无碰撞，障碍物贴近第二段（在 d_safe 内），
    // 模拟真实场景——JPS 前端保证拐点不撞障，优化器负责把轨迹推离
    std::vector<Eigen::Vector2d> corners = {
        {0.0, 0.0}, {6.0, 0.0}, {6.0, 5.0}, {9.0, 5.0}};
    opt.obstacle = Eigen::Vector2d(5.5, 2.5);

    Eigen::Matrix2Xd q;
    Eigen::VectorXd T;
    if (!navi_planner::buildMincoInitialGuess(corners, 0.4, 1.0, 0.3, 1.8, 1.5, q, T))
    {
        std::printf("[FAIL] integration: buildMincoInitialGuess returned false\n");
        return false;
    }
    const double spacing = 14.0 / (double)(q.cols() + 1);
    std::printf("  integration: corners=%d -> N=%d pieces, ctrl-pt spacing=%.3f m, sumT=%.2f s\n",
                (int)corners.size(), (int)T.size(), spacing, (double)T.sum());

    Eigen::Matrix<double, 2, 3> headPVA, tailPVA;
    headPVA << 0.0, 0.8, 0.0,   // 起点带速度（模拟重规划连续性）
        0.0, 0.0, 0.0;
    tailPVA << 9.0, 0.0, 0.0,
        5.0, 0.0, 0.0;

    minco::Trajectory<5> traj;
    const bool ok = opt.optimize(headPVA, tailPVA, q, T, traj);
    if (!ok)
    {
        std::printf("[FAIL] integration: optimize returned false\n");
        return false;
    }

    auto path = navi_planner::sampleTrajectoryByArc(traj, 0.05);
    if (path.size() < 2)
    {
        std::printf("[FAIL] integration: sampled path empty\n");
        return false;
    }
    // 弧长采样间距检查
    double maxGap = 0.0;
    for (size_t i = 1; i < path.size(); i++)
        maxGap = std::max(maxGap, (path[i] - path[i - 1]).norm());

    double minDist = 1e9, maxV = 0.0, maxA = 0.0;
    const int steps = 600;
    const double total = traj.getTotalDuration();
    for (int k = 0; k <= steps; k++)
    {
        const double t = total * k / steps;
        const Eigen::Vector2d pos = traj.getPos(t);
        minDist = std::min(minDist, std::sqrt(FakeFieldOptimizer::kEps * FakeFieldOptimizer::kEps + (pos - opt.obstacle).squaredNorm()));
        maxV = std::max(maxV, traj.getVel(t).norm());
        maxA = std::max(maxA, traj.getAcc(t).norm());
    }
    const double startErr = (path.front() - corners.front()).norm();
    const double endErr = (path.back() - corners.back()).norm();

    std::printf("  integration: cost=%.4f iters=%d time=%.1fms ret=%d\n",
                opt.lastCost(), opt.lastIterations(), opt.lastDurationMs(), opt.lastReturnCode());
    std::printf("  integration: samples=%d maxGap=%.4f minDist=%.4f maxV=%.3f maxA=%.3f endErr=%.2e\n",
                (int)path.size(), maxGap, minDist, maxV, maxA, endErr);

    if (opt.lastDurationMs() > 80.0)
    {
        std::printf("[WARN] integration: over 80ms budget on this machine\n");
    }
    if (startErr > 1e-9 || endErr > 1e-6)
    {
        std::printf("[FAIL] integration: endpoints broken\n");
        return false;
    }
    if (maxGap > 0.055)
    {
        std::printf("[FAIL] integration: arc-length sampling gap too large\n");
        return false;
    }
    // 验收标准：轨迹推离至 ~d_safe，v/a 在限幅 ×1.05 内
    if (minDist < 0.55)
    {
        std::printf("[FAIL] integration: trajectory not pushed away from obstacle (minDist=%.4f)\n", minDist);
        return false;
    }
    if (maxV > p.vmax * 1.05 || maxA > p.amax * 1.05)
    {
        std::printf("[FAIL] integration: dynamics limits violated (maxV=%.3f maxA=%.3f)\n", maxV, maxA);
        return false;
    }
    std::printf("[PASS] integration: buildMincoInitialGuess -> optimize -> arc sampling\n");
    return true;
}

int main()
{
    bool ok = true;
    ok &= test_forward();
    ok &= test_gradient();
    ok &= test_optimize();
    ok &= test_integration();
    std::printf(ok ? "ALL TESTS PASSED\n" : "TESTS FAILED\n");
    return ok ? 0 : 1;
}
