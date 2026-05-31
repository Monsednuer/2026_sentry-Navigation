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
