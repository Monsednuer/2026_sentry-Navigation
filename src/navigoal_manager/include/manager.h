#pragma once
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <cstdint>
#include <vector>
#include <utility>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace navigoal_manager
{
class GoalManager : public rclcpp::Node
{
public:
    GoalManager();
private:
    int bias;
    // std::vector<std::pair<double, double>> r_goals;
    // std::vector<std::pair<double, double>> b_goals;
    std::vector<std::pair<double, double>> r_goals;
    std::vector<std::pair<double, double>> b_goals;

    void goalCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void goalPoseCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr ly_goal_subscription_;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr ly_goal_pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
};
}  // namespace navigoal_manager    