#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

struct RegionConfig {
  std::string name;
  std::vector<double> speeds;
};

class SpeedManager : public rclcpp::Node
{
public:
  SpeedManager();

private:
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr speed_level_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr directional_region_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::TimerBase::SharedPtr pub_speed_timer_;

  std::vector<RegionConfig> regions_;
  std::string region_name_{"default"};
  std::uint8_t speed_level_{1};

  bool loadFromYaml(const std::string &config_file);
  void directionalRegionCallback(const std_msgs::msg::String::SharedPtr msg);
  void speedLevelCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void pubSpeedTimerCallback();
};
