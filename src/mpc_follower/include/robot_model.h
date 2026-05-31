#pragma once

#include <cmath>
#include <limits>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace mpc_follower
{
class MpcController;

struct RobotState
{
    double x{};
    double y{};
    double theta{};
    double vx{};
    double vy{};
    double omega{};
    rclcpp::Time stamp;
};

struct Control
{
    double vx{};
    double vy{};
};

struct PathPoint
{
    double x{};
    double y{};
    double theta{};
};

inline double min_radius = std::numeric_limits<float>::max();

inline constexpr double Ts = 0.05;
inline constexpr int N = 30;

inline constexpr double MinR = 0.0;
inline constexpr double MaxLinear = 2.0;
inline constexpr double MinLinear = -2.0;
inline constexpr double MaxAngular = 2.0;
inline constexpr double MinAngular = -2.0;

inline constexpr double MaxAccelLinear = 1.5;
inline constexpr double MinAccelLinear = -1.5;
inline constexpr double MaxAccelAngular = 6.0;
inline constexpr double MinAccelAngular = -6.0;

inline constexpr double Wx = 20.0;
inline constexpr double Wy = 20.0;
inline constexpr double Wtheta = 10.0;
inline constexpr double Wv = 0.02;
inline constexpr double Womega = 0.1;
inline constexpr double WRadius = 1.0;
inline constexpr double kStopPenalty = 0.0;
inline constexpr double Rv = 0.1;
inline constexpr double Romega = 0.1;

inline constexpr double OptTolerance = 1e-3;
inline constexpr int GlobalMaxEval = 500;
inline constexpr int LocalMaxEval = 1000;
inline constexpr double GlobalInitNoise = 0.2;

inline constexpr double ObstacleInflation = 0.5;
inline constexpr double Wobs = 100.0;

struct DynamicObstacle
{
    double x{};
    double y{};
    double radius{};
};
}
