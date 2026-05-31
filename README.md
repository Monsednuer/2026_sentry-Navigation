# navigation

## ESDF 调试与验证（步骤） 🔧

如果你修改或怀疑 `esdf` 计算有问题（例如索引转置、非方形地图下结果异常等），请按下面的步骤逐项排查：

1. 编译并启用校验宏（临时）
   - 在 `src/path_searching/src/esdf_map.cpp` 中打开调试宏：
     - 确认文件顶部有 `#define DEBUG_ESDF`（只在调试时启用）。
   - 重新构建：
     - `colcon build --packages-select path_searching --cmake-clean-cache`

2. 运行基准并观察输出（快速检测）
   - 运行已编译的基准二进制：
     - `./install/path_searching/lib/path_searching/esdf_benchmark [width] [height] [density] [Npos] [repeats]`
   - 示例：`/home/peony/navigation_ws/install/path_searching/lib/path_searching/esdf_benchmark 200 250 0.05 10 5`
   - 关注输出：
     - `更新距离场耗时：... ms`（全图重计算时间）
     - `getDist 总耗时：...` / `getNearestObstacleIndex 总耗时：...`（查询性能）
     - 如出现 `[ESDF] WARNING: validation FAILED, mismatches=...`，表示随机样本与暴力基线存在超出容差的差异。

3. 理解并调整容差
   - 目前基线验证采用以下容差：
     - 距离容差：0.2 像素
     - 索引容差：1 像素
   - 若差异在容差内，一般可认为差异是数值近似或像素化语义导致（OpenCV 的 distanceTransform 近似），无需进一步修复。
   - 若差异显著（大量 mismatches 或差值远超容差），请继续下一步调试。

4. 深度排查（若失败）
   - 检查索引约定：
     - 本项目 `esdf` 内部采用 **row-major**：`bin_map[row][col]`。
     - OpenCV `Mat::at(row,col)` 中行 = y，列 = x。必须在 `esdf_map.cpp` 中保持一致的行列转换。
   - 手动在 `esdf_map.cpp::computeDistanceField()` 中打印少量点（`showpt()`）并使用 `cv::imwrite` 输出 `test.png` 可视化检查障碍和标注是否对齐。

5. 结束调试
   - 调试确认无误后：
     - 注释掉或移除 `#define DEBUG_ESDF`（避免运行时性能/日志影响）。
     - 若需要更稳健的检查，可将验证逻辑改成仅在 CI 或开发构建中启用。

6. 建议（未来改进）
   - 将验证开关和容差做成可配置选项（命令行参数 / 环境变量）。
   - 增加自动化单元测试覆盖非方形地图与多种障碍密度，纳入 CI 校验流程。

---

# 2026_sentry
