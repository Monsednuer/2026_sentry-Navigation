#include "esdf_map.h"

#include <cmath>
#include <cstring>
#include <limits>

namespace ESDF_enviroment
{
namespace {

void writeVoronoiDebugImage(
    const std::string& output_path,
    int rows,
    int cols,
    bool** bin_map,
    navi_planner::DynamicVoronoi& voronoi_map,
    const cv::Mat* base_img = nullptr)
{
    (void)output_path;
    cv::Mat vis;
    if (base_img != nullptr && !base_img->empty() && base_img->rows >= rows && base_img->cols >= cols)
    {
        vis = base_img->clone();
        if (vis.rows != rows || vis.cols != cols)
        {
            vis = vis(cv::Rect(0, 0, cols, rows)).clone();
        }
    }
    else
    {
        vis = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
    }

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            if (voronoi_map.isVoronoi(r, c))
            {
                vis.at<cv::Vec3b>(r, c)[0] = 0;
                vis.at<cv::Vec3b>(r, c)[1] = 255;
                vis.at<cv::Vec3b>(r, c)[2] = 0;
            }
            if (bin_map[r][c])
            {
                vis.at<cv::Vec3b>(r, c)[0] = 0;
                vis.at<cv::Vec3b>(r, c)[1] = 0;
                vis.at<cv::Vec3b>(r, c)[2] = 0;
            }
        }
    }

    // cv::imwrite(output_path, vis);  // disabled for runtime performance
}

}  // namespace


double esdf::height_conversion(unsigned char height_)
{
    return ((double)height_ / 80.0f) - 1.0f;
}

void esdf::esdf_init(bool* bin_map_, int sizeY_, int sizeX_, Eigen::Vector2d offset_, bool enable_downstairs)
{
    this->enable_downstairs = enable_downstairs;
    height_map_path = "/home/hustlyrm/source/navigation/src/path_searching/maps/";

    Size[0] = sizeY_;
    Size[1] = sizeX_;
    Offset = offset_;

    bin_map = new bool*[Size[0]];
    static_bin_map = new bool*[Size[0]];
    height_map = new double*[Size[0]];
    valid_map = new bool*[Size[0]];

    for (int r = 0; r < Size[0]; r++)
    {
        bin_map[r] = new bool[Size[1]];
        static_bin_map[r] = new bool[Size[1]];
        height_map[r] = new double[Size[1]];
        valid_map[r] = new bool[Size[1]];

        std::memcpy(bin_map[r], bin_map_ + Size[1] * r, Size[1] * sizeof(bool));
        std::memcpy(static_bin_map[r], bin_map_ + Size[1] * r, Size[1] * sizeof(bool));
    }

    voronoi_map.initializeMap(Size[0], Size[1], bin_map);
    voronoi_map.update();

    img = cv::imread(height_map_path + "height_map.png");
    if (img.empty())
    {
        for (int r = 0; r < Size[0]; ++r)
        {
            for (int c = 0; c < Size[1]; ++c)
            {
                height_map[r][c] = 0.0;
                valid_map[r][c] = true;
            }
        }
        writeVoronoiDebugImage(height_map_path + "test.png", Size[0], Size[1], bin_map, voronoi_map, nullptr);
        return;
    }

    for (int r = 0; r < Size[0] && r < img.rows; r++)
    {
        for (int c = 0; c < Size[1] && c < img.cols; c++)
        {
            height_map[r][c] = height_conversion(img.at<cv::Vec3b>(r, c)[0]);
            valid_map[r][c] = img.at<cv::Vec3b>(r, c)[2] < 128;
            img.at<cv::Vec3b>(r, c)[1] = voronoi_map.isVoronoi(r, c) ? 255 : 0;
        }
    }
    writeVoronoiDebugImage(height_map_path + "test.png", Size[0], Size[1], bin_map, voronoi_map, &img);
}

void esdf::computeDistanceField()
{
    voronoi_map.update();
}

void esdf::updateDistanceField()
{
    voronoi_map.update();
}

void esdf::setDynamicObstacles(const std::vector<Eigen::Vector2i>& dynamic_cells)
{
    for (const auto& idx : last_dynamic_obstacles_)
    {
        const int r = idx[0];
        const int c = idx[1];
        if (r < 0 || c < 0 || r >= Size[0] || c >= Size[1]) continue;
        if (static_bin_map[r][c]) continue;

        if (bin_map[r][c])
        {
            bin_map[r][c] = false;
            voronoi_map.clearCell(r, c);
        }
    }
    last_dynamic_obstacles_.clear();

    for (const auto& idx : dynamic_cells)
    {
        const int r = idx[0];
        const int c = idx[1];
        if (r < 0 || c < 0 || r >= Size[0] || c >= Size[1]) continue;
        if (static_bin_map[r][c]) continue;

        if (!bin_map[r][c])
        {
            bin_map[r][c] = true;
            voronoi_map.occupyCell(r, c);
        }
        last_dynamic_obstacles_.push_back(idx);
    }

    voronoi_map.update();
    writeVoronoiDebugImage(height_map_path + "test.png", Size[0], Size[1], bin_map, voronoi_map, img.empty() ? nullptr : &img);
}

bool esdf::checkCollision(Eigen::Vector2i pos_)
{
    if (pos_[0] < 0 || pos_[1] < 0 || pos_[0] >= Size[0] || pos_[1] >= Size[1])
    {
        return true;
    }
    return bin_map[pos_[0]][pos_[1]];
}

bool esdf::checkUpStairs(Eigen::Vector2i pos_, Eigen::Vector2i next_pos_)
{
    double height_1 = height_map[pos_[0]][pos_[1]];
    double height_2 = height_map[next_pos_[0]][next_pos_[1]];

    double height_delta = height_2 - height_1;
    if (!enable_downstairs)
        height_delta = std::abs(height_delta);

    return height_delta > 0.08f;
}

double esdf::getDist(Eigen::Vector2i pos_)
{
    if (pos_[0] < 0 || pos_[1] < 0 || pos_[0] >= Size[0] || pos_[1] >= Size[1])
    {
        return std::numeric_limits<double>::infinity();
    }
    return voronoi_map.data[pos_[0]][pos_[1]].dist;
}

double esdf::getDynamicCost(Eigen::Vector2i pos_)
{
    if (pos_[0] < 0 || pos_[1] < 0 || pos_[0] >= Size[0] || pos_[1] >= Size[1])
    {
        return 0.0;
    }
    // Dynamic penalty is only for runtime-added obstacles, not static map obstacles.
    return (bin_map[pos_[0]][pos_[1]] && !static_bin_map[pos_[0]][pos_[1]]) ? 1.0 : 0.0;
}

Eigen::Vector2i esdf::getNearestObstacleIndex(Eigen::Vector2i pos_)
{
    Eigen::Vector2i idx(-1, -1);
    if (pos_[0] < 0 || pos_[1] < 0 || pos_[0] >= Size[0] || pos_[1] >= Size[1])
    {
        return idx;
    }

    int ox = voronoi_map.data[pos_[0]][pos_[1]].obstX;
    int oy = voronoi_map.data[pos_[0]][pos_[1]].obstY;
    if (ox < 0 || oy < 0 || ox >= Size[0] || oy >= Size[1])
    {
        return idx;
    }
    idx[0] = ox;
    idx[1] = oy;
    return idx;
}

esdf::esdf() {}

esdf::~esdf()
{
    if (bin_map)
    {
        for (int r = 0; r < Size[0]; r++) delete[] bin_map[r];
        delete[] bin_map;
    }
    if (static_bin_map)
    {
        for (int r = 0; r < Size[0]; r++) delete[] static_bin_map[r];
        delete[] static_bin_map;
    }
    if (height_map)
    {
        for (int r = 0; r < Size[0]; r++) delete[] height_map[r];
        delete[] height_map;
    }
    if (valid_map)
    {
        for (int r = 0; r < Size[0]; r++) delete[] valid_map[r];
        delete[] valid_map;
    }
}

} // namespace ESDF_enviroment
