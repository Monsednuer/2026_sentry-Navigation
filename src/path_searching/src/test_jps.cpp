// test_jps.cpp — JPS + 时间分配离线单测（不依赖 ROS）
#include <cstdio>
#include <cmath>
#include <random>
#include <chrono>
#include <vector>
#include "esdf_map.h"
#include "jps.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("[FAIL] %s\n", msg); failures++; } } while (0)

// 沿折线按步长采样查碰撞（步长单位：格）
static bool pathCollisionFree(ESDF_enviroment::esdf& env,
                              const std::vector<Eigen::Vector2d>& wps, double step_cells)
{
    for (size_t i = 0; i + 1 < wps.size(); ++i)
    {
        const double len = (wps[i + 1] - wps[i]).norm();
        const int steps = std::max(1, (int)std::ceil(len / (step_cells * ESDF_enviroment::esdf::kResolution)));
        for (int k = 0; k <= steps; ++k)
        {
            Eigen::Vector2d p = wps[i] + ((double)k / steps) * (wps[i + 1] - wps[i]);
            if (env.checkCollision(env.Pos2index(p))) return false;
        }
    }
    return true;
}

int main()
{
    const double res = ESDF_enviroment::esdf::kResolution;

    // ---------- 场景1：随机障碍图 400x400 (20m x 20m) ----------
    {
        const int W = 400, H = 400;
        std::vector<char> mapc(W * H, 0);
        bool* map = reinterpret_cast<bool*>(mapc.data());
        std::mt19937 rng(42);
        std::uniform_real_distribution<double> ud(0.0, 1.0);
        for (int r = 5; r < H - 5; ++r)
            for (int c = 5; c < W - 5; ++c)
                map[r * W + c] = (ud(rng) < 0.08);

        ESDF_enviroment::esdf env;
        env.esdf_init(map, H, W, Eigen::Vector2d(0, 0), false);
        navi_planner::JPS jps;
        jps.setEnvironment(ESDF_enviroment::Ptr(&env, [](ESDF_enviroment::esdf*) {}));

        Eigen::Vector2d start(1.0, 1.0), goal(18.0, 18.0);
        std::vector<Eigen::Vector2d> wps;
        auto t0 = std::chrono::steady_clock::now();
        bool ok = jps.search(start, goal, wps);
        auto t1 = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        printf("[随机图] search=%d 拐点数=%zu 扩展跳点=%d 耗时=%.3f ms\n",
               ok, wps.size(), jps.lastExpandedCount(), ms);
        CHECK(ok, "随机图应找到路径");
        if (ok)
        {
            CHECK(wps.size() >= 2, "至少含起终点");
            CHECK((wps.front() - start).norm() < 2 * res, "起点对齐");
            CHECK((wps.back() - goal).norm() < 2 * res, "终点对齐");
            CHECK(pathCollisionFree(env, wps, 0.1), "路径无碰撞(0.1格步长,含防切角)");
        }

        // 时间分配
        auto timed = navi_planner::allocateTimeTrapezoid(wps, 1.0, 0.3, 1.8, 1.5, 0.1);
        CHECK(timed.size() >= 2, "时间分配输出非空");
        bool mono = true;
        for (size_t i = 1; i < timed.size(); ++i)
            if (timed[i].t < timed[i - 1].t - 1e-9) mono = false;
        CHECK(mono, "时间戳单调递增");
        CHECK((timed.back().pos - goal).norm() < 1e-6, "时间分配终点对齐");
        printf("[时间分配] 采样点=%zu 总时长=%.2f s\n", timed.size(), timed.back().t);

        // 等弧长重采样
        auto dense = navi_planner::resampleByArcLength(wps, 0.05);
        CHECK(dense.size() > wps.size(), "密采样点数应大于拐点数");
        double maxgap = 0.0;
        for (size_t i = 1; i < dense.size(); ++i)
            maxgap = std::max(maxgap, (dense[i] - dense[i - 1]).norm());
        CHECK(maxgap < 0.075, "密采样间距不超 1.5x ds");
        printf("[重采样] 密采样点=%zu 最大间距=%.4f m\n", dense.size(), maxgap);
    }

    // ---------- 场景2：峡谷（平行双墙，穿走廊） ----------
    {
        const int W = 400, H = 400;
        std::vector<char> mapc(W * H, 0);
        bool* map = reinterpret_cast<bool*>(mapc.data());
        for (int r = 0; r < H; ++r) { map[r * W + 190] = true; map[r * W + 211] = true; }

        ESDF_enviroment::esdf env;
        env.esdf_init(map, H, W, Eigen::Vector2d(0, 0), false);
        navi_planner::JPS jps;
        jps.setEnvironment(ESDF_enviroment::Ptr(&env, [](ESDF_enviroment::esdf*) {}));

        // 起点在墙左，终点在墙右，必须穿走廊——但墙全高，应无解
        std::vector<Eigen::Vector2d> wps;
        bool blocked = jps.search(Eigen::Vector2d(5.0, 10.0), Eigen::Vector2d(15.0, 10.0), wps);
        CHECK(!blocked, "全高墙阻断应返回无解");
        printf("[峡谷-阻断] 正确返回无解: %d\n", !blocked);

        // 走廊内直行
        bool through = jps.search(Eigen::Vector2d(9.7, 10.0), Eigen::Vector2d(10.4, 10.0), wps);
        CHECK(through, "走廊内应有解");
        if (through)
            CHECK(pathCollisionFree(env, wps, 0.1), "走廊路径无碰撞");
        printf("[峡谷-走廊] search=%d 拐点数=%zu\n", through, wps.size());
    }

    // ---------- 场景3：切角 + ESDF 膨胀专项（对齐 DDR-opt 语义） ----------
    {
        // 单个障碍格 (49,50)，起终点在其对角两侧：
        //   无膨胀 → 标准切角 JPS 直接对角穿过（允许擦角，折线近直线）
        //   膨胀 0.30 → 障碍周边距离 < 半径的区域不可行，路径绕行且保持间距
        //   起点贴近障碍 → 起终点收缩使搜索仍有解
        const int W = 100, H = 100;
        std::vector<char> mapc(W * H, 0);
        bool* map = reinterpret_cast<bool*>(mapc.data());
        map[49 * W + 50] = true;  // 障碍格 (row 49, col 50)

        ESDF_enviroment::esdf env;
        env.esdf_init(map, H, W, Eigen::Vector2d(0, 0), false);
        navi_planner::JPS jps;
        jps.setEnvironment(ESDF_enviroment::Ptr(&env, [](ESDF_enviroment::esdf*) {}));

        std::vector<Eigen::Vector2d> wps;

        // (a) 无膨胀（默认）：应找到路径且 bin 无碰撞
        bool ok0 = jps.search(Eigen::Vector2d(2.25, 2.25), Eigen::Vector2d(2.75, 2.75), wps);
        CHECK(ok0, "无膨胀:切角JPS应有解");
        if (ok0) CHECK(pathCollisionFree(env, wps, 0.1), "无膨胀:路径无碰撞");
        printf("[切角-无膨胀] search=%d 拐点数=%zu\n", ok0, wps.size());

        // (b) 膨胀 0.30：路径仍应有解（绕行），且任意采样点 ESDF 距离不低于有效膨胀半径（容差半格）
        jps.setInflateRadius(0.30);
        bool ok1 = jps.search(Eigen::Vector2d(2.25, 2.25), Eigen::Vector2d(2.75, 2.75), wps);
        CHECK(ok1, "膨胀0.30:应仍有解(绕行)");
        if (ok1)
        {
            double min_d = std::numeric_limits<double>::infinity();
            for (size_t i = 0; i + 1 < wps.size(); ++i)
            {
                const double len = (wps[i + 1] - wps[i]).norm();
                const int steps = std::max(1, (int)std::ceil(len / (0.1 * res)));
                for (int k = 0; k <= steps; ++k)
                {
                    Eigen::Vector2d p = wps[i] + ((double)k / steps) * (wps[i + 1] - wps[i]);
                    min_d = std::min(min_d, env.getDistBilinear(p));
                }
            }
            // 起终点 ESDF≈0.32m，收缩后有效半径≈0.256；路径最小距离应接近该值（容差 1 格）
            CHECK(min_d > 0.20, "膨胀0.30:路径最小ESDF距离应保持~0.25m");
            printf("[切角-膨胀0.30] 拐点数=%zu 路径最小ESDF距离=%.3f m\n", wps.size(), min_d);
        }

        // (c) 起点收缩：起点 (2.45,2.40)（cell(48,49)，ESDF≈0.07m），配置 0.30 应收缩至 ~0.056 仍有解
        bool ok2 = jps.search(Eigen::Vector2d(2.45, 2.40), Eigen::Vector2d(2.75, 2.75), wps);
        CHECK(ok2, "起点贴墙:起终点收缩后应有解");
        printf("[起点收缩] search=%d 拐点数=%zu\n", ok2, wps.size());

        jps.setInflateRadius(0.0);  // 还原默认，避免影响后续场景
    }

    printf(failures == 0 ? "\n全部通过\n" : "\n失败 %d 项\n", failures);
    return failures == 0 ? 0 : 1;
}
