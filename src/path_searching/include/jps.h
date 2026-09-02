// jps.h
// 2D Jump Point Search 前端搜索（报告5.5.3，替换 A* 前端）
// 输入膨胀二值栅格（复用 esdf::bin_map / checkCollision），输出拐点序列（米制坐标）
// 附带报告5.5.3.2 思路一的时间分配（梯形加减速）与等弧长重采样工具
#ifndef JPS_H
#define JPS_H

#include "esdf_map.h"
#include <Eigen/Core>
#include <vector>
#include <limits>

namespace navi_planner {

class JPS
{
public:
    void setEnvironment(ESDF_enviroment::Ptr env) { env_ = env; }

    // start/goal: 米制地图坐标 (x, y)
    // waypoints: 输出拐点序列（含起点终点），米制地图坐标
    // 返回 false = 无路径 / 起终点非法
    bool search(const Eigen::Vector2d& start, const Eigen::Vector2d& goal,
                std::vector<Eigen::Vector2d>& waypoints);

    // 调试用：返回最近一次搜索的扩展跳点数
    int lastExpandedCount() const { return last_expanded_; }

private:
    ESDF_enviroment::Ptr env_;
    int last_expanded_ = 0;

    // true = 占据或出界
    bool occupied(int r, int c) const;

    // 从 (r,c) 沿 (dr,dc) 跳：命中目标/强制邻居点返回 true 并置 (jr,jc)，否则 false
    bool jump(int r, int c, int dr, int dc, int gr, int gc, int& jr, int& jc) const;

    // 是否有强制邻居（当前点即跳点）
    bool hasForced(int r, int c, int dr, int dc) const;

    static double heuristic(int r, int c, int gr, int gc)
    {
        const double dx = std::abs((double)c - gc);
        const double dy = std::abs((double)r - gr);
        const double diag = std::min(dx, dy);
        return (dx + dy - diag) + (1.4142135623730951 - 1.0) * diag;
    }
};

// ===================== 时间分配（报告5.5.3.2 思路一） =====================
struct TimedPoint
{
    Eigen::Vector2d pos;  // 米制
    double t = 0.0;       // 从轨迹起点计时的时刻（s）
};

// JPS 拐点折线 -> 考虑转角的等效路程 s = k1*s1 + k2*s2
// -> 梯形加减速总时长 -> 按 dt 均匀时间重采样
std::vector<TimedPoint> allocateTimeTrapezoid(
    const std::vector<Eigen::Vector2d>& waypoints,
    double k1, double k2, double vmax, double amax, double dt);

// 折线等弧长重采样（密采样发 /sPath 用），ds 单位米
std::vector<Eigen::Vector2d> resampleByArcLength(
    const std::vector<Eigen::Vector2d>& waypoints, double ds);

}  // namespace navi_planner

#endif
