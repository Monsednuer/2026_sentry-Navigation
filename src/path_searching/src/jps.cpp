// jps.cpp
// 2D Jump Point Search 实现 + 梯形加减速时间分配
#include "jps.h"

#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm>

namespace navi_planner
{
namespace
{
struct OpenEntry
{
    double f;
    double g;
    int r, c;
    bool operator>(const OpenEntry& o) const { return f > o.f; }
};

inline int64_t keyOf(int r, int c) { return (int64_t)r << 32 | (uint32_t)c; }
}  // namespace

bool JPS::occupied(int r, int c) const
{
    if (!env_) return true;
    Eigen::Vector2i idx(r, c);
    return env_->checkCollision(idx);  // 出界即占据（checkCollision 内部已处理）
}

bool JPS::hasForced(int r, int c, int dr, int dc) const
{
    if (dr != 0 && dc != 0)
    {
        // 对角：侧向受阻且其前方自由 -> 强制邻居
        if (occupied(r - dr, c + dc) && !occupied(r - dr, c)) return true;
        if (occupied(r + dr, c - dc) && !occupied(r, c - dc)) return true;
        return false;
    }
    if (dr != 0)
    {
        // 竖直移动：上下(行方向)前进，检查列方向两侧
        if (occupied(r, c + 1) && !occupied(r + dr, c + 1)) return true;
        if (occupied(r, c - 1) && !occupied(r + dr, c - 1)) return true;
        return false;
    }
    // 水平移动
    if (occupied(r + 1, c) && !occupied(r + 1, c + dc)) return true;
    if (occupied(r - 1, c) && !occupied(r - 1, c + dc)) return true;
    return false;
}

bool JPS::jump(int r, int c, int dr, int dc, int gr, int gc, int& jr, int& jc) const
{
    const int nr = r + dr;
    const int nc = c + dc;

    if (occupied(nr, nc)) return false;
    // 防切角：对角移动要求两个正交方向均自由
    if (dr != 0 && dc != 0)
    {
        if (occupied(nr, c) || occupied(r, nc)) return false;
    }

    if (nr == gr && nc == gc)
    {
        jr = nr;
        jc = nc;
        return true;
    }

    if (hasForced(nr, nc, dr,dc))
    {
        jr = nr;
        jc = nc;
        return true;
    }

    if (dr != 0 && dc != 0)
    {
        // 对角方向：先向两个剆链各视一审，录中刖米等注，jump主什了一次
        int tr, tc;
        if (jump(nr, nc, dr, 0, gr, gc, tr, tc) || jump(nr, nc, 0, dc, gr, gc, tr, tc))
        {
            jr = nr;
            jc = nc;
            return true;
        }
    }

    return jump(nr, nc, dr, dc, gr, gc, jr, jc);
}

bool JPS::search(const Eigen::Vector2d& start, const Eigen::Vector2d& goal,
                 std::vector<Eigen::Vector2d>& waypoints)
{
    waypoints.clear();
    last_expanded_ = 0;
    if (!env_) return false;

    const Eigen::Vector2i s = env_->Pos2index(start);   // index: (row, col)
    const Eigen::Vector2i g = env_->Pos2index(goal);
    if (occupied(s[0], s[1]) || occupied(g[0], g[1])) return false;

    std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>> open;
    // key -> (g, parent_key)
    std::unordered_map<int64_t, double> gscore;
    std::unordered_map<int64_t, int64_t> parent;
    std::unordered_map<int64_t, bool> closed;

    const int64_t sk = keyOf(s[0], s[1]);
    const int64_t gk = keyOf(g[0], g[1]);
    gscore[sk] = 0.0;
    parent[sk] = -1;
    open.push({heuristic(s[0], s[1], g[0], g[1]), 0.0, s[0], s[1]});

    static const int DIRS[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    bool found = false;
    while (!open.empty())
    {
        const OpenEntry cur = open.top();
        open.pop();
        const int64_t ck = keyOf(cur.r, cur.c);
        if (closed[ck]) continue;
        closed[ck] = true;
        last_expanded_++;

        if (ck == gk)
        {
            found = true;
            break;
        }

        for (const auto& d : DIRS)
        {
            const int dr = d[0], dc = d[1];
            // 防切角：对角扩展需两正交方向自由
            if (dr != 0 && dc != 0)
            {
                if (occupied(cur.r + dr, cur.c) || occupied(cur.r, cur.c + dc)) continue;
            }
            int jr, jc;
            if (!jump(cur.r, cur.c, dr, dc, g[0], g[1], jr, jc)) continue;

            const int64_t jk = keyOf(jr, jc);
            if (closed[jk]) continue;

            const double step = std::hypot((double)(jr - cur.r), (double)(jc - cur.c));
            const double ng = cur.g + step;
            auto it = gscore.find(jk);
            if (it == gscore.end() || ng < it->second - 1e-9)
            {
                gscore[jk] = ng;
                parent[jk] = ck;
                open.push({ng + heuristic(jr, jc, g[0], g[1]), ng, jr, jc});
            }
        }
    }

    if (!found) return false;

    // 回溯跳点（天然即拐点，无需共线裁剪）
    std::vector<Eigen::Vector2i> cells;
    int64_t k = gk;
    while (k != -1)
    {
        cells.emplace_back((int)(k >> 32), (int)(uint32_t)k);
        k = parent[k];
    }
    std::reverse(cells.begin(), cells.end());

    waypoints.reserve(cells.size());
    for (const auto& idx : cells)
        waypoints.push_back(env_->Index2pos(idx));
    return true;
}

// ===================== 时间分配 =====================
std::vector<TimedPoint> allocateTimeTrapezoid(
    const std::vector<Eigen::Vector2d>& waypoints,
    double k1, double k2, double vmax, double amax, double dt)
{
    std::vector<TimedPoint> out;
    const size_t n = waypoints.size();
    if (n == 0) return out;
    if (n == 1)
    {
        out.push_back({waypoints[0], 0.0});
        return out;
    }

    // 各段长度与拐点转角
    std::vector<double> seglen(n - 1, 0.0);
    std::vector<double> corner(n, 0.0);  // corner[i]: 第 i 个拐点处的转角（rad）
    double s1 = 0.0, s2 = 0.0;
    for (size_t i = 0; i + 1 < n; ++i)
    {
        seglen[i] = (waypoints[i + 1] - waypoints[i]).norm();
        s1 += seglen[i];
    }
    for (size_t i = 1; i + 1 < n; ++i)
    {
        Eigen::Vector2d a = waypoints[i] - waypoints[i - 1];
        Eigen::Vector2d b = waypoints[i + 1] - waypoints[i];
        if (a.norm() < 1e-9 || b.norm() < 1e-9) continue;
        double cosang = a.dot(b) / (a.norm() * b.norm());
        cosang = std::min(1.0, std::max(-1.0, cosang));
        corner[i] = std::acos(cosang);
        s2 += corner[i];
    }

    // 等效路程累计：段内按 k1*len 线性累积，转角惩罚 k2*angle 挂在拐点处
    // cum[i]: 到达第 i 个拐点时的等效路程（含该拐点转角惩罚）
    std::vector<double> cum(n, 0.0);
    for (size_t i = 1; i < n; ++i)
        cum[i] = cum[i - 1] + k1 * seglen[i - 1] + k2 * corner[i - 1];
    const double s_eq = cum[n - 1] + k2 * corner[n - 1];  // 终点转角为0，防御性累加

    if (s_eq < 1e-9)
    {
        out.push_back({waypoints[0], 0.0});
        return out;
    }

    // 梯形/三角速度剖面总时长
    double T, vpeak;
    const double s_reach = vmax * vmax / amax;  // 加减速各 vmax²/(2a)，合计 vmax²/a
    if (s_eq <= s_reach)
    {
        vpeak = std::sqrt(s_eq * amax);
        T = 2.0 * vpeak / amax;
    }
    else
    {
        vpeak = vmax;
        T = s_eq / vmax + vmax / amax;
    }

    // 等效路程 s(t) -> 折线位置
    auto eqToPos = [&](double s) -> Eigen::Vector2d {
        // 找到 s 落在哪个拐点到拐点区间
        size_t i = 0;
        while (i + 1 < n && cum[i + 1] < s) ++i;
        if (i + 1 >= n) return waypoints[n - 1];
        const double seg_eq = k1 * seglen[i];
        if (seg_eq < 1e-12) return waypoints[i + 1];
        // 区间内等效路程从 cum[i]（已含起点转角）线性到 cum[i]+seg_eq
        double frac = (s - cum[i]) / seg_eq;
        frac = std::min(1.0, std::max(0.0, frac));
        return waypoints[i] + frac * (waypoints[i + 1] - waypoints[i]);
    };

    const double t_acc = vpeak / amax;
    for (double t = 0.0; t <= T + 1e-9; t += dt)
    {
        const double tt = std::min(t, T);
        double s;
        if (tt <= t_acc)
            s = 0.5 * amax * tt * tt;
        else if (tt <= T - t_acc)
            s = 0.5 * amax * t_acc * t_acc + vpeak * (tt - t_acc);
        else
        {
            const double td = tt - (T - t_acc);
            s = s_eq - 0.5 * amax * (t_acc - td) * (t_acc - td);
        }
        s = std::min(s_eq, std::max(0.0, s));
        out.push_back({eqToPos(s), tt});
    }
    if (out.empty() || (out.back().pos - waypoints[n - 1]).norm() > 1e-6)
        out.push_back({waypoints[n - 1], T});
    return out;
}

std::vector<Eigen::Vector2d> resampleByArcLength(
    const std::vector<Eigen::Vector2d>& waypoints, double ds)
{
    std::vector<Eigen::Vector2d> out;
    const size_t n = waypoints.size();
    if (n == 0) return out;
    if (ds <= 1e-9 || n == 1)
    {
        out.push_back(waypoints[0]);
        return out;
    }
    out.push_back(waypoints[0]);
    double carry = 0.0;  // 上一段剩余未采样长度
    for (size_t i = 0; i + 1 < n; ++i)
    {
        const Eigen::Vector2d a = waypoints[i];
        const Eigen::Vector2d b = waypoints[i + 1];
        const double len = (b - a).norm();
        if (len < 1e-12) continue;
        double d = ds - carry;
        while (d < len)
        {
            out.push_back(a + (d / len) * (b - a));
            d += ds;
        }
        carry = len - (d - ds);
    }
    if ((out.back() - waypoints[n - 1]).norm() > 1e-6)
        out.push_back(waypoints[n - 1]);
    return out;
}

}  // namespace navi_planner
