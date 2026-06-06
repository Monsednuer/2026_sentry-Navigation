#pragma once

#include "robot_model.h"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <nlopt.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

namespace mpc_follower
{
class MpcController : public rclcpp::Node
{
public:
    explicit MpcController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    Control computeControl();
    void publishVisualization();
    void configureGlobalOptimizer();
    void configureLocalOptimizer();
    void controlLoop();
    void cmdVelPublish(const Control &u);
    bool updateRobotState();
    void onReferencePath(const nav_msgs::msg::Path::SharedPtr msg);
    void detectObstacles(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void detectObstacleUpdates(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg);
    void setSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg);
    int findNearestWaypoint(const RobotState &state, size_t start_index);

    static double objectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *data);
    static double accelerationConstraints(const std::vector<double> &x, std::vector<double> &grad, void *data);
    static double turningRadiusConstraint(const std::vector<double> &x, std::vector<double> &grad, void *data);

    RobotState predictState(const RobotState &current, const Control &u, double dt, MpcController *mpc);

private:
    struct Params
    {
        double Ts{};
        double MaxLinear{};
        double MinLinear{};
        double MaxAngular{};
        double MinAngular{};
        double MaxAccelLinear{};
        double MinAccelLinear{};
        double MaxAccelAngular{};
        double MinAccelAngular{};
        double MinR{};
        double Wx{};
        double Wy{};
        double Wtheta{};
        double Wv{};
        double Womega{};
        double WRadius{};
        double Rv{};
        double Romega{};
        double TargetSpeed{};
        double ReachSlowDistance{};
        double ReachStopDistance{};
        double OptTolerance{};
        int GlobalMaxEval{};
        int LocalMaxEval{};
        double GlobalInitNoise{};
        double ObstacleInflation{};
        double Wobs{};
        int N{};
        bool UseTurningRadius{};
        std::string GlobalFrame{"map"};
        std::string BaseFrame{"base_link"};
    } params_;

    void loadParameters();
    void updateReachedState();
    Control applySpeedLimit(const Control &u) const;
    int normalizeCostmapCellToOcc100(int8_t raw_cell) const;
    double costmapObstacleCost(double x, double y) const;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr obstacle_update_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    RobotState current_state_;
    Control previous_control_;
    std::vector<PathPoint> reference_path_;
    bool has_state_{false};
    bool has_prev_tf_state_{false};
    bool reached_flag_{false};
    double prev_tf_x_{0.0};
    double prev_tf_y_{0.0};
    rclcpp::Time prev_tf_stamp_;

    nlopt::opt global_optimizer_;
    nlopt::opt local_optimizer_;
    std::vector<double> opt_result_;
    double opt_fval_{};

    size_t last_index_{0};
    std::mt19937 rng_;
    nav_msgs::msg::MapMetaData latest_costmap_info_;
    std::vector<int8_t> latest_costmap_data_;
    bool has_costmap_{false};
};
}
