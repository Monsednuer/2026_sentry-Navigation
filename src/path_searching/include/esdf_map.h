#ifndef ESDF_H
#define ESDF_H

#include "dynamicvoronoi.h"
#include <Eigen/Core>
#include <memory>
#include <opencv2/highgui.hpp>
#include <string>
#include <vector>

namespace ESDF_enviroment {

class esdf
{
public:
    bool** bin_map = nullptr;
    bool** static_bin_map = nullptr;
    bool** valid_map = nullptr;
    double** height_map = nullptr;
    cv::Mat img;
    Eigen::Vector2i Size;
    Eigen::Vector2d Offset;
    bool enable_downstairs = false;
    std::string height_map_path;
    navi_planner::DynamicVoronoi voronoi_map;

public:
    void esdf_init(bool* bin_map_, int sizeX_, int sizeY_, Eigen::Vector2d offset_, bool enable_downstairs = false);
    void computeDistanceField();
    void updateDistanceField();

    void setDynamicObstacles(const std::vector<Eigen::Vector2i>& dynamic_cells);

    esdf();
    ~esdf();
    double height_conversion(unsigned char height_);
    bool checkCollision(Eigen::Vector2i pos_);
    bool checkUpStairs(Eigen::Vector2i pos_, Eigen::Vector2i next_pos_);
    double getDist(Eigen::Vector2i pos_);
    double getDynamicCost(Eigen::Vector2i pos_);
    Eigen::Vector2i getNearestObstacleIndex(Eigen::Vector2i pos_);
    void getMapRegion(Eigen::Vector2i & map_size_){map_size_ =  Size;}

    // ===================== 连续坐标插值查询（报告5.5.2.2） =====================
    // 栅格分辨率（米/格），与 Index2pos/Pos2index 中的 20.0 对应
    static constexpr double kResolution = 0.05;
    // 输入：地图坐标系位置（米）；输出距离单位：米；梯度无量纲（方向远离障碍物）。
    // 采样点为 Cell 中心，cell (r,c) 中心对应 ((c+0.5)*res+ox, (r+0.5)*res+oy)。
    // 双线性插值（基线，用于对比；峡谷脊线处梯度不连续，存在梯度无效化问题）
    double getDistBilinear(const Eigen::Vector2d& pos_m) const;
    Eigen::Vector2d getGradBilinear(const Eigen::Vector2d& pos_m) const;
    // 双二次 Lagrange 插值（3x3 邻域，每个方向三点拟合二次函数）：
    // 距离与梯度均解析、平滑，解决峡谷形障碍物中间梯度无效化问题
    double getDistQuadratic(const Eigen::Vector2d& pos_m) const;
    Eigen::Vector2d getGradQuadratic(const Eigen::Vector2d& pos_m) const;

    Eigen::Vector2d Index2pos(Eigen::Vector2i index_)
    {
        Eigen::Vector2d index_d((double)index_[1], (double)index_[0]);
        return index_d / 20.0f + Offset;
    }

    Eigen::Vector2i Pos2index(Eigen::Vector2d pos)
    {
        Eigen::Vector2d index_ = (pos - Offset) * 20.0f;
        Eigen::Vector2i index((int)index_[1], (int)index_[0]);
        return index;
    }

private:
    std::vector<Eigen::Vector2i> last_dynamic_obstacles_;
};

typedef std::shared_ptr<esdf> Ptr;

}

#endif
