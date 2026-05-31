#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

struct RegionConfig {
  std::string name;
  std::vector<geometry_msgs::msg::Point> polygon;
  geometry_msgs::msg::Point direction_start;
  geometry_msgs::msg::Point direction_end;
  bool active = false;
};

class RegionDetector : public rclcpp::Node
{
public:
  RegionDetector();

private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr speed_level_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr directional_region_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_rotated_pub_;

  std::vector<RegionConfig> regions_;
  std::vector<geometry_msgs::msg::Point> path_points_;
  geometry_msgs::msg::Point current_position_{};

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::uint8_t speed_level_{1};

  bool loadFromYaml(const std::string &config_file);
  bool isPointInPolygon(
    const geometry_msgs::msg::Point &pt,
    const std::vector<geometry_msgs::msg::Point> &poly) const;
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void poseTimerCallback();
  void speedLevelCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void checkLogic();
  void checkRotation();
  void publishMarkers();
};
