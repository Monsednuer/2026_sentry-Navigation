// traj_optimizer.cpp
// MINCO 单阶段轨迹优化实现（第三步）
#include "traj_optimizer.h"

#include "esdf_map.h"
#include "jps.h"
#include "minco/lbfgs.hpp"

#include <cmath>
#include <iostream>
#include <limits>

namespace navi_planner {

namespace {
// 段时间软屏障边界（T = exp(τ)）：防止时长发散导致带状方程组病态
constexpr double kTauMin = -3.0;  // T >= ~0.05 s
constexpr double kTauMax = 3.4;   // T <= ~30 s
constexpr double kBarrierW = 1.0e6;
}  // namespace

void TrajOptimizer::setEnvironment(ESDF_enviroment::Ptr env) { env_ = env; }

void TrajOptimizer::setParam(const TrajOptParam &p) { param_ = p; }

double TrajOptimizer::queryDist(const Eigen::Vector2d &p) const
{
    if (!env_) return 1.0e3;
    return env_->getDistQuadratic(p);
}

Eigen::Vector2d TrajOptimizer::queryGrad(const Eigen::Vector2d &p) const
{
    if (!env_) return Eigen::Vector2d::Zero();
    return env_->getGradQuadratic(p);
}

double TrajOptimizer::costFuncCallback(void *instance, const Eigen::VectorXd &x, Eigen::VectorXd &g)
{
    return static_cast<TrajOptimizer *>(instance)->evaluate(x, g);
}

int TrajOptimizer::progressCallback(void *instance, const Eigen::VectorXd &,
                                    const Eigen::VectorXd &, const double,
                                    const double, const int k, const int)
{
    auto *self = static_cast<TrajOptimizer *>(instance);
    self->last_iters_ = k;
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - self->t0_).count();
    if (elapsed_ms > self->param_.max_time_ms)
    {
        self->timed_out_ = true;
        return 1;  // 取消迭代，保留当前已接受的最优点
    }
    return 0;
}

// J(q, τ)，T = exp(τ)；梯度经 MINCO 伴随回传
// 变量打包：x = [ q_x(N-1) | q_y(N-1) | τ(N) ]
double TrajOptimizer::evaluate(const Eigen::VectorXd &x, Eigen::VectorXd &g)
{
    const int Nm = N_ - 1;
    Eigen::Matrix2Xd q(2, Nm);
    q.row(0) = x.segment(0, Nm).transpose();
    q.row(1) = x.segment(Nm, Nm).transpose();

    Eigen::VectorXd tau = x.segment(2 * Nm, N_);
    Eigen::VectorXd T(N_);
    double barrier_cost = 0.0;
    Eigen::VectorXd barrier_grad_tau = Eigen::VectorXd::Zero(N_);
    for (int i = 0; i < N_; i++)
    {
        // 软屏障：保持 T 在合理区间，避免带状系统病态
        if (tau(i) < kTauMin)
        {
            const double d = tau(i) - kTauMin;
            barrier_cost += kBarrierW * d * d;
            barrier_grad_tau(i) += 2.0 * kBarrierW * d;
            tau(i) = kTauMin;
        }
        else if (tau(i) > kTauMax)
        {
            const double d = tau(i) - kTauMax;
            barrier_cost += kBarrierW * d * d;
            barrier_grad_tau(i) += 2.0 * kBarrierW * d;
            tau(i) = kTauMax;
        }
        T(i) = std::exp(tau(i));
    }

    solver_.setParameters(q, T);

    // ---------- 能量项 ----------
    double energy = 0.0;
    solver_.getEnergy(energy);
    Eigen::MatrixX2d gdC;
    Eigen::VectorXd gdT;
    solver_.getEnergyPartialGradByCoeffs(gdC);
    solver_.getEnergyPartialGradByTimes(gdT);
    double cost = param_.w_energy * energy;
    gdC *= param_.w_energy;
    gdT *= param_.w_energy;

    // ---------- 采样点惩罚项（避障 / 速度 / 加速度） ----------
    const Eigen::MatrixX2d &coeffs = solver_.getCoeffs();
    const double vmax2 = param_.vmax * param_.vmax;
    const double amax2 = param_.amax * param_.amax;

    for (int i = 0; i < N_; i++)
    {
        const double Ti = T(i);
        const int K = std::max(1, static_cast<int>(std::ceil(Ti / param_.sample_dt)));
        for (int k = 1; k <= K; k++)
        {
            const double t = Ti * k / K;
            // p, v, a, j 与系数基函数（c_m 对应 coeffs.row(6i+m)）
            Eigen::Vector2d p = Eigen::Vector2d::Zero();
            Eigen::Vector2d v = Eigen::Vector2d::Zero();
            Eigen::Vector2d a = Eigen::Vector2d::Zero();
            Eigen::Vector2d j = Eigen::Vector2d::Zero();
            double posB[6], velB[6], accB[6];
            double tm = 1.0;
            for (int m = 0; m < 6; m++)
            {
                const Eigen::Vector2d cm = coeffs.row(6 * i + m).transpose();
                posB[m] = tm;
                p += tm * cm;
                // velB[m] = m*t^(m-1)，accB[m] = m(m-1)*t^(m-2)，用已算好的 t 幂次复用
                velB[m] = (m >= 1) ? m * posB[m - 1] : 0.0;
                accB[m] = (m >= 2) ? m * (m - 1) * posB[m - 2] : 0.0;
                v += velB[m] * cm;
                a += accB[m] * cm;
                if (m >= 3)
                {
                    j += m * (m - 1) * (m - 2) * posB[m - 3] * cm;
                }
                tm *= t;
            }

            Eigen::Vector2d gp = Eigen::Vector2d::Zero();
            Eigen::Vector2d gv = Eigen::Vector2d::Zero();
            Eigen::Vector2d ga = Eigen::Vector2d::Zero();

            // 避障：C(d) = (d_safe - d)^3，d < d_safe
            const double d = queryDist(p);
            if (d < param_.esdf_grad_radius)
            {
                const bool fine_stage =
                    param_.two_stage && stage_ == OptStage::FINELY_OPTIMIZATION;
                if (!fine_stage)
                {
                    // PRE / 单阶段：ESDF 原始梯度直接用（现状语义）
                    const double r = param_.esdf_grad_radius - d;
                    cost += param_.w_collision * r * r * r;
                    gp += -3.0 * param_.w_collision * r * r * queryGrad(p);
                }
                else
                {
                    // [MINCO_V2] FINELY 阶段避障准梯度（报告5.5.4.2）：
                    //   法向分解（去切向）+ 可移动性探测 + 势谷分支。
                    //   准梯度非任何解析 cost 的精确梯度，L-BFGS 面向分段光滑设计可容纳。
                    const double spd = v.norm();
                    if (spd > 1e-6)
                    {
                        const Eigen::Vector2d vhat = v / spd;  // 运动切向单位向量
                        const Eigen::Vector2d g_esdf = queryGrad(p);
                        const Eigen::Vector2d gN = g_esdf - (g_esdf.dot(vhat)) * vhat;  // 法向梯度
                        if (gN.norm() > 1e-9)
                        {
                            const Eigen::Vector2d ghat = gN / gN.norm();  // 单位化法向
                            // 可移动性探测：沿法向探一步看距离场是否放开
                            const double d_probe =
                                queryDist(p + param_.fine_probe_step * ghat);
                            const double g_probe = (d_probe - d) / param_.fine_probe_step;
                            if (g_probe >= param_.fine_grad_threshold)
                            {
                                // 自由分支：cost 同 PRE 三次惩罚，方向为法向单位化（报告 "normalize"）
                                const double r = param_.esdf_grad_radius - d;
                                cost += param_.w_collision * r * r * r;
                                gp += -3.0 * param_.w_collision * r * r * ghat;
                            }
                            else
                            {
                                // 势谷分支：viola = fine_scale*sqrt(||gN||)，继续推会优化到另一端
                                const double viola =
                                    param_.fine_scale * std::sqrt(gN.norm());
                                cost += param_.w_collision * viola * viola * viola;
                                gp += -3.0 * param_.w_collision * viola * viola * ghat;
                            }
                        }
                        // ||gN|| <= 1e-9：门中线两侧梯度对消，跳过该点避障梯度
                    }
                    // ||v|| <= 1e-6：无法定义运动法向，跳过该点避障梯度
                }
            }

            // 速度越限：P = (||v||^2 - vmax^2)^2
            const double v2 = v.squaredNorm();
            if (v2 > vmax2)
            {
                const double u = v2 - vmax2;
                cost += param_.w_vel * u * u;
                gv += 4.0 * param_.w_vel * u * v;
            }

            // 加速度越限：P = (||a||^2 - amax^2)^2
            const double a2 = a.squaredNorm();
            if (a2 > amax2)
            {
                const double u = a2 - amax2;
                cost += param_.w_acc * u * u;
                ga += 4.0 * param_.w_acc * u * a;
            }

            // 采样时刻 t = Ti*k/K 对 Ti 的显式依赖：dt/dTi = t/Ti
            gdT(i) += (gp.dot(v) + gv.dot(a) + ga.dot(j)) * (t / Ti);

            for (int m = 0; m < 6; m++)
            {
                gdC.row(6 * i + m) += posB[m] * gp.transpose() +
                                      velB[m] * gv.transpose() +
                                      accB[m] * ga.transpose();
            }
        }
    }

    cost += barrier_cost;

    // [MINCO_V2] 时间正则（two_stage 模式下 PRE 与 FINELY 两阶段都生效；two_stage=false 不生效 = 第三步回归）：
    //   绝对时间形式（报告5.5.4.2，与 DDR-opt 同构）：T̄ = (ΣTᵢ)/N 用 barrier 夹紧后的 T 计算；
    //   超界段 eᵢ = Tᵢ − bound·T̄，∂c/∂Tⱼ = Σᵢ 2·w·eᵢ·(δᵢⱼ − boundᵢ/N)（T̄ 对所有 Tⱼ 有依赖，梯度稠密）。
    //   经 T=exp(τ) 链式在下方统一乘 T。该项解析可微，纳入 PRE 阶段中心差分校验。
    //   注：报告写"仅第一阶段需要"，但 FINELY 的能量项会把段时间重新拉歪（实测 V1 版 T-ratio 漂到 [0.62,1.50]，
    //   集成场景 [0.71,3.50]）——DDR-opt 原码也是两阶段都施加 mean-time 正则（optimizer.cpp L961-995 / L1534-1545），故两阶段都开。
    if (param_.two_stage)
    {
        Eigen::VectorXd reg_grad_T = Eigen::VectorXd::Zero(N_);  // ∂c_reg/∂T
        const double Tbar = T.mean();
        double reg_cost = 0.0;
        for (int i = 0; i < N_; i++)
        {
            double bound = 0.0, e = 0.0;
            if (T(i) > param_.time_reg_upper * Tbar)
            {
                e = T(i) - param_.time_reg_upper * Tbar;
                bound = param_.time_reg_upper;
            }
            else if (T(i) < param_.time_reg_lower * Tbar)
            {
                e = T(i) - param_.time_reg_lower * Tbar;
                bound = param_.time_reg_lower;
            }
            if (e != 0.0)
            {
                reg_cost += param_.w_time_reg * e * e;
                const double diag = 2.0 * param_.w_time_reg * e;
                reg_grad_T(i) += diag;                    // δᵢⱼ 项
                reg_grad_T.array() -= diag * bound / N_;  // −boundᵢ/N 项（稠密）
            }
        }
        cost += reg_cost;
        gdT += reg_grad_T;
    }

    // ---------- 伴随回传 -> (q, T) 梯度 ----------
    Eigen::Matrix2Xd gradQ;
    Eigen::VectorXd gradT;
    solver_.propogateGrad(gdC, gdT, gradQ, gradT);

    g.resize(3 * N_ - 2);
    g.segment(0, Nm) = gradQ.row(0).transpose();
    g.segment(Nm, Nm) = gradQ.row(1).transpose();
    g.segment(2 * Nm, N_) = gradT.cwiseProduct(T) + barrier_grad_tau;

    return cost;
}

void TrajOptimizer::prepare(const Eigen::Matrix<double, 2, 3> &headPVA,
                            const Eigen::Matrix<double, 2, 3> &tailPVA,
                            int pieceNum)
{
    N_ = pieceNum;
    head_pva_ = headPVA;
    tail_pva_ = tailPVA;
    solver_.setConditions(head_pva_, tail_pva_, N_);
}

double TrajOptimizer::evaluateCost(const Eigen::VectorXd &x, Eigen::VectorXd &g)
{
    if (N_ < 1 || x.size() != 3 * N_ - 2)
    {
        std::cerr << "[TrajOptimizer] evaluateCost: call prepare() first / bad x size" << std::endl;
        return std::numeric_limits<double>::quiet_NaN();
    }
    return evaluate(x, g);
}

bool TrajOptimizer::optimize(const Eigen::Matrix<double, 2, 3> &headPVA,
                             const Eigen::Matrix<double, 2, 3> &tailPVA,
                             const Eigen::Matrix2Xd &initInnerPts,
                             const Eigen::VectorXd &initT,
                             minco::Trajectory<5> &outTraj)
{
    last_ret_ = 0;
    last_iters_ = 0;
    last_ret_pre_ = 0;   // [MINCO_V2]
    last_iters_pre_ = 0; // [MINCO_V2]
    last_ms_pre_ = 0.0;  // [MINCO_V2]
    timed_out_ = false;
    const auto wall0 = std::chrono::steady_clock::now();

    N_ = static_cast<int>(initT.size());
    const int Nm = N_ - 1;
    if (N_ < 1 || initInnerPts.cols() != Nm)
    {
        std::cerr << "[TrajOptimizer] bad input size: N=" << N_
                  << " innerPts=" << initInnerPts.cols() << std::endl;
        return false;
    }
    for (int i = 0; i < N_; i++)
    {
        if (!(initT(i) > 0.0) || !std::isfinite(initT(i)))
        {
            std::cerr << "[TrajOptimizer] invalid initial T(" << i << ")=" << initT(i) << std::endl;
            return false;
        }
    }

    head_pva_ = headPVA;
    tail_pva_ = tailPVA;
    solver_.setConditions(head_pva_, tail_pva_, N_);

    Eigen::VectorXd x(3 * N_ - 2);
    x.segment(0, Nm) = initInnerPts.row(0).transpose();
    x.segment(Nm, Nm) = initInnerPts.row(1).transpose();
    for (int i = 0; i < N_; i++)
    {
        x(2 * Nm + i) = std::log(initT(i));
    }

    // 记录初值代价：若优化后代价显著变差（震荡发散），视为失败让上层回退折线。
    // [MINCO_V2] 护栏语义统一：两阶段模式最终 cost 为 FINELY 语义，故初值也用 FINELY 语义估一次；
    //            单阶段（two_stage=false）沿用第三步语义（PRE 原始梯度、无时间正则）。
    stage_ = param_.two_stage ? OptStage::FINELY_OPTIMIZATION : OptStage::PRE_OPTIMIZATION;
    Eigen::VectorXd g0;
    const double init_cost = evaluate(x, g0);

    // [MINCO_V2] delta 提为参数 param_.lbfgs_delta（原硬编码 1e-5 过紧；DDR-opt 实配 5e-3）
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = 8;
    lbfgs_params.g_epsilon = 1.0e-5;
    lbfgs_params.past = 3;
    lbfgs_params.delta = param_.lbfgs_delta;
    lbfgs_params.max_linesearch = 32;

    t0_ = std::chrono::steady_clock::now();  // [MINCO_V2] 两阶段共享同一墙钟（progressCallback 超时取消覆盖全程）
    double minf = 0.0;
    int ret = 0;

    if (param_.two_stage)
    {
        // [MINCO_V2] ---------- PRE 阶段（含时间正则）：稳定轨迹形状（推出障碍物） ----------
        lbfgs_params.max_iterations = param_.max_iter_pre;
        const auto tPre0 = std::chrono::steady_clock::now();
        stage_ = OptStage::PRE_OPTIMIZATION;
        ret = lbfgs::lbfgs_optimize(x, minf,
                                    &TrajOptimizer::costFuncCallback,
                                    nullptr,
                                    &TrajOptimizer::progressCallback,
                                    this,
                                    lbfgs_params);
        const auto tPre1 = std::chrono::steady_clock::now();
        last_ret_pre_ = ret;
        last_iters_pre_ = last_iters_;  // progressCallback 已写入本阶段最终迭代数
        last_ms_pre_ = std::chrono::duration<double, std::milli>(tPre1 - tPre0).count();
        // 阶段间不做护栏：PRE 结果直接喂 FINELY（即使 PRE 返回容忍码也继续，形值已可用）
        if (std::getenv("MINCO_DBG"))
        {
            double s = 0.0, mn = 1e9, mx = 0.0;
            for (int i = 0; i < N_; i++)
            {
                const double t = std::exp(std::min(std::max(x(2 * Nm + i), kTauMin), kTauMax));
                s += t;
            }
            const double mb = s / N_;
            for (int i = 0; i < N_; i++)
            {
                const double t = std::exp(std::min(std::max(x(2 * Nm + i), kTauMin), kTauMax));
                mn = std::min(mn, t / mb);
                mx = std::max(mx, t / mb);
            }
            std::printf("[DBG PRE ] ret=%d iters=%d T-ratio=[%.3f,%.3f] Tmean=%.3f\n",
                        ret, last_iters_pre_, mn, mx, mb);
        }

        // [MINCO_V2] ---------- FINELY 阶段（法向梯度 + 势谷，无时间正则） ----------
        // 独立调用 ⇒ L-BFGS 有限记忆 / pf 历史全部重置 = 换 cost 地形后的重启（非续跑）
        lbfgs_params.max_iterations = param_.max_iter_fine;
        stage_ = OptStage::FINELY_OPTIMIZATION;
        ret = lbfgs::lbfgs_optimize(x, minf,
                                    &TrajOptimizer::costFuncCallback,
                                    nullptr,
                                    &TrajOptimizer::progressCallback,
                                    this,
                                    lbfgs_params);
        last_iters_ = last_iters_pre_ + last_iters_;  // 全程总迭代（两阶段各自计数之和）
        last_ret_ = ret;                              // 最终阶段（FINELY）返回码
        last_cost_ = minf;
        last_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall0).count();
        if (std::getenv("MINCO_DBG"))
        {
            double s = 0.0, mn = 1e9, mx = 0.0;
            for (int i = 0; i < N_; i++)
            {
                const double t = std::exp(std::min(std::max(x(2 * Nm + i), kTauMin), kTauMax));
                s += t;
            }
            const double mb = s / N_;
            for (int i = 0; i < N_; i++)
            {
                const double t = std::exp(std::min(std::max(x(2 * Nm + i), kTauMin), kTauMax));
                mn = std::min(mn, t / mb);
                mx = std::max(mx, t / mb);
            }
            std::printf("[DBG FINE] ret=%d fine_iters=%d T-ratio=[%.3f,%.3f] Tmean=%.3f cost=%.6f\n",
                        ret, last_iters_ - last_iters_pre_, mn, mx, mb, minf);
        }
    }
    else
    {
        // [MINCO_V2] 单阶段回退开关：two_stage=false ⇒ 无时间正则、ESDF 原始梯度 ⇒ 与第三步完全一致
        lbfgs_params.max_iterations = param_.max_iter;
        stage_ = OptStage::PRE_OPTIMIZATION;
        ret = lbfgs::lbfgs_optimize(x, minf,
                                    &TrajOptimizer::costFuncCallback,
                                    nullptr,
                                    &TrajOptimizer::progressCallback,
                                    this,
                                    lbfgs_params);
        last_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall0).count();
        last_cost_ = minf;
        last_ret_ = ret;
    }

    // L-BFGS 对线搜索失败会把 x 回退到上一个已接受点，轨迹依然可用（第四步打补丁根治）
    const bool tolerated =
        ret == lbfgs::LBFGSERR_MAXIMUMLINESEARCH ||
        ret == lbfgs::LBFGSERR_MINIMUMSTEP ||
        ret == lbfgs::LBFGSERR_MAXIMUMSTEP ||
        ret == lbfgs::LBFGSERR_WIDTHTOOSMALL ||
        ret == lbfgs::LBFGSERR_MAXIMUMITERATION;
    if (ret < 0 && !tolerated)
    {
        std::cerr << "[TrajOptimizer] L-BFGS failed: " << lbfgs::lbfgs_strerror(ret) << std::endl;
        return false;
    }
    if (ret < 0)
    {
        std::cout << "[TrajOptimizer] L-BFGS tolerated non-success: "
                  << lbfgs::lbfgs_strerror(ret) << std::endl;
    }
    if (!std::isfinite(minf))
    {
        std::cerr << "[TrajOptimizer] non-finite cost" << std::endl;
        return false;
    }

    // 安全护栏：优化后代价不应显著差于初值（回退到上个接受点也不允许变差）。
    // 若显著变差说明本次优化震荡/发散，宁可回退折线（第二步行为），不发布更差的轨迹。
    if (std::isfinite(init_cost) && minf > init_cost * 1.5 && minf > init_cost + 1.0)
    {
        std::cerr << "[TrajOptimizer] cost worsened after optimize ("
                  << init_cost << " -> " << minf << "), reject" << std::endl;
        return false;
    }

    // 用最终变量重建轨迹（evaluate 已在 solver_ 中留下最后一次求值状态，这里显式重建保证一致）
    Eigen::Matrix2Xd q(2, Nm);
    q.row(0) = x.segment(0, Nm).transpose();
    q.row(1) = x.segment(Nm, Nm).transpose();
    Eigen::VectorXd T(N_);
    for (int i = 0; i < N_; i++)
    {
        const double tauc = std::min(std::max(x(2 * Nm + i), kTauMin), kTauMax);
        T(i) = std::exp(tauc);
    }
    solver_.setParameters(q, T);
    solver_.getTrajectory(outTraj);

    // 基本可用性检查：时长为正、位置有限
    for (int i = 0; i < outTraj.getPieceNum(); i++)
    {
        if (!(outTraj[i].getDuration() > 1.0e-3)) return false;
        const Eigen::Vector2d p0 = outTraj[i].getPos(0.0);
        const Eigen::Vector2d p1 = outTraj[i].getPos(outTraj[i].getDuration());
        if (!p0.allFinite() || !p1.allFinite()) return false;
    }
    return true;
}

std::vector<Eigen::Vector2d> sampleTrajectoryByArc(const minco::Trajectory<5> &traj, double ds)
{
    std::vector<Eigen::Vector2d> dense;
    if (traj.getPieceNum() == 0 || ds <= 0.0) return dense;
    for (int i = 0; i < traj.getPieceNum(); i++)
    {
        const double dur = traj[i].getDuration();
        const int n = std::max(4, static_cast<int>(std::ceil(dur / 0.02)));
        const int k0 = (i == 0) ? 0 : 1;
        for (int k = k0; k <= n; k++)
        {
            dense.push_back(traj[i].getPos(dur * k / n));
        }
    }
    return resampleByArcLength(dense, ds);
}

bool buildMincoInitialGuess(const std::vector<Eigen::Vector2d> &corners,
                            double ctrl_pt_interval,
                            double k1, double k2, double vmax, double amax,
                            Eigen::Matrix2Xd &innerPts, Eigen::VectorXd &initT)
{
    const size_t M = corners.size();
    if (M < 2) return false;

    // ---- 折线弧长参数化 ----
    std::vector<double> seg_len(M - 1, 0.0);
    double total_len = 0.0;
    for (size_t i = 0; i + 1 < M; i++)
    {
        seg_len[i] = (corners[i + 1] - corners[i]).norm();
        total_len += seg_len[i];
    }
    if (total_len < 1e-6) return false;

    auto polyPointAtArc = [&](double s) -> Eigen::Vector2d {
        double acc = 0.0;
        for (size_t i = 0; i + 1 < M; i++)
        {
            if (s <= acc + seg_len[i] || i + 2 == M)
            {
                const double frac = seg_len[i] > 1e-9
                    ? std::min(1.0, std::max(0.0, (s - acc) / seg_len[i]))
                    : 0.0;
                return corners[i] + frac * (corners[i + 1] - corners[i]);
            }
            acc += seg_len[i];
        }
        return corners.back();
    };

    // ---- 中间控制点初值 ----
    const int n_inner = std::max(1,
        static_cast<int>(std::lround(total_len / std::max(0.1, ctrl_pt_interval))) - 1);
    innerPts.resize(2, n_inner);
    for (int k = 1; k <= n_inner; k++)
    {
        innerPts.col(k - 1) = polyPointAtArc(total_len * k / (n_inner + 1));
    }

    // ---- 段时间初值：前端梯形时间分配的累计时间 ----
    auto timed = allocateTimeTrapezoid(corners, k1, k2, vmax, amax, 0.05);
    initT.resize(n_inner + 1);
    if (timed.size() >= 2)
    {
        // timed 采样点 -> (累计弧长, 时间) 序列，再按弧长插值出各控制点时刻
        std::vector<double> s_of_t(timed.size(), 0.0);
        for (size_t i = 1; i < timed.size(); i++)
        {
            s_of_t[i] = s_of_t[i - 1] + (timed[i].pos - timed[i - 1].pos).norm();
        }
        const double s_total = s_of_t.back();
        auto timeAtArc = [&](double s) -> double {
            if (s_total < 1e-9) return 0.0;
            const double ss = std::min(s_total, std::max(0.0, s * s_total / total_len));
            size_t i = 0;
            while (i + 1 < timed.size() && s_of_t[i + 1] < ss) i++;
            if (i + 1 >= timed.size()) return timed.back().t;
            const double ds = s_of_t[i + 1] - s_of_t[i];
            const double frac = ds > 1e-9 ? (ss - s_of_t[i]) / ds : 0.0;
            return timed[i].t + frac * (timed[i + 1].t - timed[i].t);
        };
        double t_prev = 0.0;
        for (int k = 1; k <= n_inner + 1; k++)
        {
            const double s_k = (k == n_inner + 1) ? total_len : total_len * k / (n_inner + 1);
            const double t_k = timeAtArc(s_k);
            initT(k - 1) = std::max(0.1, t_k - t_prev);
            t_prev = t_k;
        }
    }
    else
    {
        // 防御：均匀时间分配
        const double avg = (total_len / std::max(0.5, vmax)) / (n_inner + 1);
        initT.setConstant(std::max(0.2, avg));
    }
    return true;
}

}  // namespace navi_planner
