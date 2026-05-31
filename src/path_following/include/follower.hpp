#ifndef PATH_FOLLOWER_FOLLOWER_HPP
#define PATH_FOLLOWER_FOLLOWER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <algorithm>

namespace path_follower
{

class Follower : public rclcpp::Node
{
public:
    Follower();

private:
    // Core parameters
    double control_dt_;
    double speed_dt_;
    double speed_ratio_;
    double kp_, ki_, kd_;
    double k_lookahead_, min_lookahead_;
    double target_speed_;
    double lookahead_distance_;
    int current_index_;
    bool reached_flag_ = 0;

    // Data structures
    std::vector<Eigen::Vector2d> global_path_;
    std::vector<Eigen::Vector2d> local_path_;
    Eigen::Vector2d target_velocity_;
    Eigen::Vector2d control_velocity_;
    Eigen::Vector2d current_velocity_local_;
    Eigen::Vector2d current_velocity_direction_;
    Eigen::Vector2d follow_velocity_;
    Eigen::Vector2d target_goal_local_;
    Eigen::MatrixXd map_baselink_tf_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS 2 Interfaces
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_path_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_set_speed_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
    //TEST
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_vel_dir_map_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_reached_;
    
    rclcpp::TimerBase::SharedPtr timer_manager_;
    rclcpp::TimerBase::SharedPtr timer_get_current_velocity_;

    // Methods
    void manager();
    void getCurrentVelocity();
    bool getTransform();
    void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void setSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg);
    
    void getLookaheadDistance();
    void getLocalPath();
    void getCurrentIndex();
    void checkReached();
    void getTargetVelocity();
    void follow();
    void control();
    void publish();
    void visualize();
};

} // namespace path_follower

#endif // PATH_FOLLOWER_FOLLOWER_HPP