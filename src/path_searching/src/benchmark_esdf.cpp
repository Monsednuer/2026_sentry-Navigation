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
    int mismatches = 0;
    for (int i = 0; i < baseline_N; i++)
    {
        Eigen::Vector2i p = samples[i];
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
    if (mismatches > 0)
    {
        std::cout << "[ESDF 基准] 验证失败，不匹配数=" << mismatches << std::endl;
        return 1;
    }
    else
    {
        std::cout << "[ESDF 基准] 验证通过" << std::endl;
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
    return 0;
}
