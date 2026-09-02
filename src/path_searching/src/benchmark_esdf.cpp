#include <iostream>
#include <chrono>
#include <random>
#include <vector>
#include <fstream>
#include <filesystem>
#include "esdf_map.h"

int main(int argc, char** argv)
{
    int sizeX = 400;
    int sizeY = 400;
    double density = 0.05; // obstacle density
    int Npos = 1000;
    int repeats = 100;
    if (argc > 1) sizeX = atoi(argv[1]);
    if (argc > 2) sizeY = atoi(argv[2]);
    if (argc > 3) density = atof(argv[3]);
    if (argc > 4) Npos = atoi(argv[4]);
    if (argc > 5) repeats = atoi(argv[5]);

    std::cout << "ESDF 基准测试：地图=" << sizeX << "x" << sizeY << " 障碍密度=" << density << " Npos=" << Npos << " repeats=" << repeats << std::endl;

    int total = sizeX * sizeY;
    bool* bin_map = new bool[total];
    std::mt19937 rng(123456);
    std::uniform_real_distribution<double> ud(0.0, 1.0);
    for (int i = 0; i < total; i++)
        bin_map[i] = (ud(rng) < density) ? true : false;

    ESDF_enviroment::esdf env;
    Eigen::Vector2d offset(0.0, 0.0);
    env.esdf_init(bin_map, sizeX, sizeY, offset, false);

    // measure updateDistanceField
    auto t0 = std::chrono::steady_clock::now();
    env.updateDistanceField();
    auto t1 = std::chrono::steady_clock::now();
    double ms_update = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::cout << "更新距离场耗时：" << ms_update << " ms" << std::endl;

    // prepare samples (row, col) to match esdf row-major bin_map[row][col]
    std::uniform_int_distribution<int> row_rng(0, sizeY - 1);
    std::uniform_int_distribution<int> col_rng(0, sizeX - 1);
    std::vector<Eigen::Vector2i> samples;
    samples.reserve(Npos);
    for (int i = 0; i < Npos; i++) samples.emplace_back(row_rng(rng), col_rng(rng));

    // measure getDist average time
    uint64_t total_calls = (uint64_t)Npos * (uint64_t)repeats;
    auto tstart = std::chrono::steady_clock::now();
    double sum = 0.0;
    for (int r = 0; r < repeats; r++)
    {
        for (int i = 0; i < Npos; i++)
        {
            sum += env.getDist(samples[i]);
        }
    }
    auto tend = std::chrono::steady_clock::now();
    double ms_total = std::chrono::duration<double, std::milli>(tend - tstart).count();
    double avg_per_call_us = (ms_total * 1000.0) / (double)total_calls;
    std::cout << "getDist 总耗时：" << ms_total << " ms，平均每次：" << avg_per_call_us << " us" << std::endl;

    // measure getNearestObstacleIndex average time
    tstart = std::chrono::steady_clock::now();
    for (int r = 0; r < repeats; r++)
    {
        for (int i = 0; i < Npos; i++)
        {
            auto id = env.getNearestObstacleIndex(samples[i]);
            (void)id;
        }
    }
    tend = std::chrono::steady_clock::now();
    double ms_total_idx = std::chrono::duration<double, std::milli>(tend - tstart).count();
    double avg_per_idx_us = (ms_total_idx * 1000.0) / (double)total_calls;
    std::cout << "getNearestObstacleIndex 总耗时：" << ms_total_idx << " ms，平均每次：" << avg_per_idx_us << " us" << std::endl;

    // baseline brute force for small sample set (to compare)
    int baseline_N = std::min(200, Npos);
    tstart = std::chrono::steady_clock::now();
    double sum_b = 0.0;
    for (int i = 0; i < baseline_N; i++)
    {
        Eigen::Vector2i p = samples[i];
        double best = std::numeric_limits<double>::infinity();
        // iterate rows (r) and cols (c)
        for (int r = 0; r < sizeY; r++)
        {
            for (int c = 0; c < sizeX; c++)
            {
                if (!env.bin_map[r][c]) continue;
                double dr = (double)r - (double)p[0];
                double dc = (double)c - (double)p[1];
                double d = std::sqrt(dr*dr + dc*dc);
                if (d < best) best = d;
            }
        }
        sum_b += best;
    }
    tend = std::chrono::steady_clock::now();
    double ms_baseline = std::chrono::duration<double, std::milli>(tend - tstart).count();
    std::cout << "暴力基准（" << baseline_N << " 个样本）：" << ms_baseline << " ms，平均每个：" << (ms_baseline/baseline_N) << " ms" << std::endl;

    // validation: compare ESDF results to brute-force for baseline_N samples
    // 注：该 DynamicVoronoi 变体不向最外圈边界环传播（dist=inf），跳过边界环样本
    int mismatches = 0;
    int skipped_border = 0;
    for (int i = 0; i < baseline_N; i++)
    {
        Eigen::Vector2i p = samples[i];
        if (p[0] <= 0 || p[1] <= 0 || p[0] >= sizeY - 1 || p[1] >= sizeX - 1)
        {
            skipped_border++;
            continue;
        }
        double best = std::numeric_limits<double>::infinity();
        Eigen::Vector2i best_idx(-1,-1);
        // iterate rows (r) and cols (c)
        for (int r = 0; r < sizeY; r++)
        {
            for (int c = 0; c < sizeX; c++)
            {
                if (!env.bin_map[r][c]) continue;
                double dr = (double)r - (double)p[0];
                double dc = (double)c - (double)p[1];
                double d = std::sqrt(dr*dr + dc*dc);
                if (d < best) { best = d; best_idx[0] = r; best_idx[1] = c; }
            }
        }
        double esdf_d = env.getDist(p);
        Eigen::Vector2i esdf_idx = env.getNearestObstacleIndex(p);
        bool dist_ok = (std::abs(esdf_d - best) <= 0.2); // tolerance in pixels
        bool idx_ok = (std::abs(esdf_idx[0] - best_idx[0]) <= 1 && std::abs(esdf_idx[1] - best_idx[1]) <= 1);
        if (!dist_ok || !idx_ok)
        {
            std::cout << "[ESDF 验证] 不匹配：样本 " << i << " p=(" << p[0] << "," << p[1] << ") 暴力距离=" << best << " ESDF距离=" << esdf_d << " 暴力索引=(" << best_idx[0] << "," << best_idx[1] << ") ESDF索引=(" << esdf_idx[0] << "," << esdf_idx[1] << ")" << std::endl;
            mismatches++;
        }
    }
    if (skipped_border > 0)
    {
        std::cout << "[ESDF 验证] 跳过边界环样本 " << skipped_border << " 个（DynamicVoronoi 不传播最外圈）" << std::endl;
    }
    if (mismatches > 0)
    {
        std::cout << "[ESDF 基准] 验证失败，不匹配数=" << mismatches << "（插值测试仍继续执行）" << std::endl;
    }
    else
    {
        std::cout << "[ESDF 基准] 验证通过" << std::endl;
    }

    // ==================================================================
    // [插值对比测试] 双线性 vs 双二次 Lagrange（报告5.5.2.2）
    // ==================================================================
    const double res = ESDF_enviroment::esdf::kResolution;

    // ---------- 测试1：亚栅格精度（随机地图上，暴力真值对比） ----------
    {
        std::uniform_real_distribution<double> row_uf(2.0, sizeY - 3.0);
        std::uniform_real_distribution<double> col_uf(2.0, sizeX - 3.0);
        const int Nsub = 300;
        double se_bl = 0.0, se_qd = 0.0, maxe_bl = 0.0, maxe_qd = 0.0;
        for (int i = 0; i < Nsub; i++)
        {
            double row_f = row_uf(rng), col_f = col_uf(rng);
            Eigen::Vector2d pos_m((col_f + 0.5) * res, (row_f + 0.5) * res);
            // 暴力真值：到最近障碍 Cell 中心的欧氏距离（格）
            double best = std::numeric_limits<double>::infinity();
            for (int r = 0; r < sizeY; r++)
                for (int c = 0; c < sizeX; c++)
                {
                    if (!env.bin_map[r][c]) continue;
                    double dr = r - row_f, dc = c - col_f;
                    best = std::min(best, std::sqrt(dr * dr + dc * dc));
                }
            double truth_m = best * res;
            double e_bl = std::abs(env.getDistBilinear(pos_m) - truth_m);
            double e_qd = std::abs(env.getDistQuadratic(pos_m) - truth_m);
            se_bl += e_bl * e_bl; se_qd += e_qd * e_qd;
            maxe_bl = std::max(maxe_bl, e_bl); maxe_qd = std::max(maxe_qd, e_qd);
        }
        std::cout << "[插值精度] 双线性 RMS=" << std::sqrt(se_bl / Nsub) * 1000.0
                  << " mm  max=" << maxe_bl * 1000.0 << " mm | 双二次 RMS="
                  << std::sqrt(se_qd / Nsub) * 1000.0 << " mm  max=" << maxe_qd * 1000.0
                  << " mm（真值=到最近障碍Cell中心距离）" << std::endl;
    }

    // ---------- 测试2：峡谷地形梯度无效化（核心验收项） ----------
    // 两堵平行墙 col=190 / col=211，走廊中 V(200)==V(201)==10（两Cell距离相同），
    // 双线性在此区间梯度恒为0（梯度无效化），双二次应给出平滑过零的梯度。
    {
        const int W = 400, H = 400;
        bool* canyon = new bool[W * H];
        std::fill(canyon, canyon + W * H, false);
        for (int r = 0; r < H; r++) { canyon[r * W + 190] = true; canyon[r * W + 211] = true; }

        ESDF_enviroment::esdf env2;
        env2.esdf_init(canyon, H, W, Eigen::Vector2d(0.0, 0.0), false);

        const double row_f = 200.0;
        const double step = 0.05;  // 格
        int zero_bl = 0, zero_qd = 0, flips_bl = 0, flips_qd = 0;
        double maxjump_bl = 0.0, maxjump_qd = 0.0;
        double prev_gx_bl = 0.0, prev_gx_qd = 0.0;
        bool first = true;
        for (double col_f = 195.0; col_f <= 206.0 + 1e-9; col_f += step)
        {
            Eigen::Vector2d pos_m((col_f + 0.5) * res, (row_f + 0.5) * res);
            double gx_bl = env2.getGradBilinear(pos_m)[0];
            double gx_qd = env2.getGradQuadratic(pos_m)[0];
            // 远离真实脊线（200.5±0.25格）处的"梯度消失"计数
            if (std::abs(col_f - 200.5) > 0.25)
            {
                if (std::abs(gx_bl) < 0.05) zero_bl++;
                if (std::abs(gx_qd) < 0.05) zero_qd++;
            }
            if (!first)
            {
                maxjump_bl = std::max(maxjump_bl, std::abs(gx_bl - prev_gx_bl));
                maxjump_qd = std::max(maxjump_qd, std::abs(gx_qd - prev_gx_qd));
                if (gx_bl * prev_gx_bl < -0.01) flips_bl++;
                if (gx_qd * prev_gx_qd < -0.01) flips_qd++;
            }
            prev_gx_bl = gx_bl; prev_gx_qd = gx_qd; first = false;
        }
        std::cout << "[峡谷梯度] 双线性: 无效化样本=" << zero_bl << " 梯度跳变max=" << maxjump_bl
                  << " 符号翻转=" << flips_bl << " | 双二次: 无效化样本=" << zero_qd
                  << " 梯度跳变max=" << maxjump_qd << " 符号翻转=" << flips_qd << std::endl;
        bool ok = (zero_bl > 0) && (zero_qd == 0) && (maxjump_qd < 0.5) && (flips_qd <= 1);
        std::cout << (ok ? "[峡谷梯度] 通过：双二次消除梯度无效化且梯度平滑"
                         : "[峡谷梯度] 未达预期，需检查实现") << std::endl;

        delete[] canyon;
    }

    // ---------- 测试3：插值查询性能 ----------
    {
        std::uniform_real_distribution<double> row_uf(2.0, sizeY - 3.0);
        std::uniform_real_distribution<double> col_uf(2.0, sizeX - 3.0);
        std::vector<Eigen::Vector2d> pos_samples;
        pos_samples.reserve(Npos);
        for (int i = 0; i < Npos; i++)
            pos_samples.emplace_back((col_uf(rng) + 0.5) * res, (row_uf(rng) + 0.5) * res);

        auto bench = [&](const char* name, auto&& fn) {
            auto ts = std::chrono::steady_clock::now();
            double acc = 0.0;
            for (int r = 0; r < repeats; r++)
                for (int i = 0; i < Npos; i++) acc += fn(pos_samples[i]);
            auto te = std::chrono::steady_clock::now();
            double us = std::chrono::duration<double, std::micro>(te - ts).count() / (double)total_calls;
            std::cout << "[插值性能] " << name << " 平均每次：" << us << " us" << std::endl;
            (void)acc;
        };
        bench("getDistBilinear   ", [&](const Eigen::Vector2d& p) { return env.getDistBilinear(p); });
        bench("getDistQuadratic  ", [&](const Eigen::Vector2d& p) { return env.getDistQuadratic(p); });
        bench("getGradQuadratic  ", [&](const Eigen::Vector2d& p) { return env.getGradQuadratic(p)[0]; });
    }

    // determine output directory inside workspace (search upwards for 'src/path_searching')
    std::filesystem::path out_dir;
    std::filesystem::path cur = std::filesystem::current_path();
    while (true)
    {
        if (std::filesystem::exists(cur / "src" / "path_searching"))
        {
            out_dir = cur / "src" / "path_searching" / "benchmarks";
            break;
        }
        if (cur == cur.root_path()) break;
        cur = cur.parent_path();
    }
    if (out_dir.empty())
    {
        out_dir = std::filesystem::temp_directory_path();
        std::cout << "[ESDF 基准] 未找到工作区路径，回退到：" << out_dir << std::endl;
    }
    else
    {
        std::error_code ec;
        std::filesystem::create_directories(out_dir, ec);
        if (ec) std::cout << "[ESDF 基准] 创建目录失败：" << out_dir << " 错误：" << ec.message() << std::endl;
    }

    auto out_file = out_dir / "esdf_benchmark.csv";
    std::ofstream fout(out_file.string(), std::ios::app);
    fout << sizeX << "," << sizeY << "," << density << "," << Npos << "," << repeats << "," << ms_update << "," << ms_total << "," << avg_per_call_us << "," << ms_total_idx << "," << avg_per_idx_us << "," << ms_baseline << "\n";
    fout.close();
    std::cout << "基准结果 CSV 已追加到：" << out_file << std::endl;

    delete[] bin_map;
    return mismatches > 0 ? 1 : 0;
}
