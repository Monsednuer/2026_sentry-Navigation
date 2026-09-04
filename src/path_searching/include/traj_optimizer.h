// traj_optimizer.h
// MINCO 轨迹优化器封装（第三步，报告5.5.3 思路二 / 技术报告 PRE_OPTIMIZATION 单阶段版）
// 输入：起终点 PVA + 中间控制点初值 + 段时间初值
// 输出：优化后的分段五次多项式轨迹（minco::Trajectory<5>）
// cost = w_energy*E(MINCO jerk能量)
//      + Σ采样点 [ w_coll*C(d(p))  (d<d_safe 时三次惩罚)
//                + w_vel*P(||v||) + w_acc*P(||a||) ]
// 时间无约束化 T=exp(τ)，全部梯度经 MINCO 伴随回传，L-BFGS 求解。
#ifndef TRAJ_OPTIMIZER_H
#define TRAJ_OPTIMIZER_H

#include "minco/minco.hpp"

#include <Eigen/Core>
#include <chrono>
#include <memory>
#include <vector>

namespace ESDF_enviroment {
class esdf;
typedef std::shared_ptr<esdf> Ptr;
}

namespace navi_planner {

struct TrajOptParam
{
    double ctrl_pt_interval = 0.4; // 控制点平均间距（m），仅初值构造使用
    double sample_dt = 0.05;       // cost 采样时间步（s）
    int max_iter = 200;            // L-BFGS 最大迭代轮次
    double max_time_ms = 80.0;     // wall clock 硬上限
    double esdf_grad_radius = 0.6; // 避障梯度介入距离 d_safe（m）
    double w_energy = 1.0;
    double w_collision = 500.0;
    double w_vel = 10.0;
    double w_acc = 10.0;
    double vmax = 1.8;
    double amax = 1.5;
};

class TrajOptimizer
{
public:
    void setEnvironment(ESDF_enviroment::Ptr env);
    void setParam(const TrajOptParam &p);
    const TrajOptParam &getParam() const { return param_; }

    // headPVA/tailPVA: 2x3（列 = 位置/速度/加速度）
    // initInnerPts: 2 x (N-1)，initT: N
    // 返回 false = 优化失败（调用方应 fallback 到折线路径）
    bool optimize(const Eigen::Matrix<double, 2, 3> &headPVA,
                  const Eigen::Matrix<double, 2, 3> &tailPVA,
                  const Eigen::Matrix2Xd &initInnerPts,
                  const Eigen::VectorXd &initT,
                  minco::Trajectory<5> &outTraj);

    // 供数值微分校验/调试：先 prepare 设定端点条件与段数，再反复调用 evaluateCost
    void prepare(const Eigen::Matrix<double, 2, 3> &headPVA,
                 const Eigen::Matrix<double, 2, 3> &tailPVA,
                 int pieceNum);
    // 单点 cost/梯度求值（需先 prepare 或 optimize），变量打包 x=[q_x|q_y|τ]
    double evaluateCost(const Eigen::VectorXd &x, Eigen::VectorXd &g);

    double lastCost() const { return last_cost_; }
    double lastDurationMs() const { return last_ms_; }
    int lastReturnCode() const { return last_ret_; }
    int lastIterations() const { return last_iters_; }

protected:
    // 距离场查询（虚函数，便于单测注入解析距离场校验梯度）
    virtual double queryDist(const Eigen::Vector2d &p) const;
    virtual Eigen::Vector2d queryGrad(const Eigen::Vector2d &p) const;

private:
    static double costFuncCallback(void *instance, const Eigen::VectorXd &x, Eigen::VectorXd &g);
    static int progressCallback(void *instance, const Eigen::VectorXd &x,
                                const Eigen::VectorXd &g, const double fx,
                                const double step, const int k, const int ls);

    double evaluate(const Eigen::VectorXd &x, Eigen::VectorXd &g);

    ESDF_enviroment::Ptr env_;
    TrajOptParam param_;
    Eigen::Matrix<double, 2, 3> head_pva_, tail_pva_;
    minco::MINCO_S3NU_2D solver_;
    int N_ = 0;
    double last_cost_ = 0.0;
    double last_ms_ = 0.0;
    int last_ret_ = 0;
    int last_iters_ = 0;
    std::chrono::steady_clock::time_point t0_;
    bool timed_out_ = false;
};

// 轨迹按弧长 ds（m）重采样为点列（含首尾），供 /sPath 发布
std::vector<Eigen::Vector2d> sampleTrajectoryByArc(const minco::Trajectory<5> &traj, double ds);

// JPS 拐点折线 -> MINCO 优化初值：
// 中间控制点按弧长约 ctrl_pt_interval 均布，段时间取梯形时间分配
// (k1, k2, vmax, amax) 的累计时间差。corners 需含起终点且 >= 2 个点。
bool buildMincoInitialGuess(const std::vector<Eigen::Vector2d> &corners,
                            double ctrl_pt_interval,
                            double k1, double k2, double vmax, double amax,
                            Eigen::Matrix2Xd &innerPts, Eigen::VectorXd &initT);

}  // namespace navi_planner

#endif
