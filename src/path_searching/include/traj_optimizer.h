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

    // [MINCO_V2] 两阶段优化参数（第四步，报告5.5.4.2；名字与 plan_manager 接线共用，勿改）
    bool two_stage = true;            // 两阶段开关；false = 第三步单阶段行为（无时间正则）
    int max_iter_pre = 1000;          // PRE 阶段迭代上限（放大上限让 delta 收敛判定退出，80ms 墙钟兜底——报告5.5.4.1"放大迭代次数上限"）
    int max_iter_fine = 1000;         // FINELY 阶段迭代上限（同上）
    double lbfgs_delta = 1.0e-3;      // [V2] 线搜索提前接受阈值（原硬编码 1e-5 过紧；DDR-opt 实配 5e-3）
    double w_time_reg = 50.0;         // 时间正则权重（仅两阶段模式的 PRE）
    double time_reg_lower = 0.9;      // 段时间/平均段时间 下限
    double time_reg_upper = 1.1;      // 段时间/平均段时间 上限
    double fine_grad_threshold = 0.5; // 势谷判定阈值（可移动性探测梯度）
    double fine_probe_step = 0.1;     // 可移动性探测步长（m，> ESDF 0.05 分辨率）
    double fine_scale = 0.6;          // 势谷 viola 尺度（默认 = d_safe 量级）
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

    // [MINCO_V2] 阶段统计 getter：PRE 阶段（two_stage=false 单阶段运行时不记录，返回 0）
    int lastRetPre() const { return last_ret_pre_; }
    int lastItersPre() const { return last_iters_pre_; }
    double lastMsPre() const { return last_ms_pre_; }

    // [MINCO_V2] 优化阶段枚举（供单测子类切换做 PRE/FINELY 分阶段梯度校验）
    enum class OptStage
    {
        PRE_OPTIMIZATION = 0,
        FINELY_OPTIMIZATION = 1
    };

protected:
    // [MINCO_V2] 当前优化阶段 + protected 切换钩子（供单测子类如 FakeFieldOptimizer 切换做分阶段梯度校验）
    OptStage stage_ = OptStage::PRE_OPTIMIZATION;
    void setStage(OptStage s) { stage_ = s; }

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
    // [MINCO_V2] PRE 阶段统计（两阶段模式记录；单阶段运行保持初值 0）
    double last_ms_pre_ = 0.0;
    int last_ret_pre_ = 0;
    int last_iters_pre_ = 0;
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
