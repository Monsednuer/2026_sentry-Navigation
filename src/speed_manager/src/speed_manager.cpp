#include "speed_manager.hpp"

#include <chrono>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

SpeedManager::SpeedManager()
: Node("speed_manager")
{
  directional_region_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/ly/navi/directional_region",
    1,
    std::bind(&SpeedManager::directionalRegionCallback, this, std::placeholders::_1));
  speed_level_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
    "/ly/navi/speed_level",
    1,
    std::bind(&SpeedManager::speedLevelCallback, this, std::placeholders::_1));

  speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("/setFollowSpeed", 1);

  std::string default_config;
  try
  {
    default_config = ament_index_cpp::get_package_share_directory("speed_manager") + "/cfg/speed.yaml";
  }
  catch (const std::exception &e)
  {
    RCLCPP_WARN(this->get_logger(), "Failed to locate package share: %s", e.what());
  }
  this->declare_parameter<std::string>("config_file", default_config);
  std::string config_file = this->get_parameter("config_file").as_string();
  if (!loadFromYaml(config_file))
  {
    RCLCPP_WARN(this->get_logger(), "Speed config not loaded, output speed will stay 0.");
  }

  pub_speed_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&SpeedManager::pubSpeedTimerCallback, this));
}

bool SpeedManager::loadFromYaml(const std::string &config_file)
{
  if (config_file.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'config_file' is empty.");
    return false;
  }

  YAML::Node root;
  try
  {
    root = YAML::LoadFile(config_file);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load config file '%s': %s", config_file.c_str(), e.what());
    return false;
  }

  const YAML::Node regions_node = root["regions"];
  if (!regions_node || !regions_node.IsSequence())
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid config '%s': key 'regions' must be a sequence.", config_file.c_str());
    return false;
  }

  regions_.clear();
  for (const auto &node : regions_node)
  {
    if (!node["name"] || !node["speeds"])
    {
      continue;
    }

    RegionConfig region;
    region.name = node["name"].as<std::string>();

    const auto speeds = node["speeds"];
    if (!speeds.IsSequence())
    {
      RCLCPP_WARN(this->get_logger(), "Region %s has invalid speeds.", region.name.c_str());
      continue;
    }
    for (const auto &speed : speeds)
    {
      region.speeds.push_back(speed.as<double>());
    }

    if (!region.speeds.empty())
    {
      regions_.push_back(region);
    }
  }

  RCLCPP_INFO(this->get_logger(), "Loaded %zu speed regions from %s", regions_.size(), config_file.c_str());
  return !regions_.empty();
}

void SpeedManager::speedLevelCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data <= 2)
  {
    speed_level_ = msg->data;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid speed level: %u. Must be 0, 1, or 2.", msg->data);
    speed_level_ = 1;
  }
}

void SpeedManager::directionalRegionCallback(const std_msgs::msg::String::SharedPtr msg)
{
  region_name_ = msg->data;
}

void SpeedManager::pubSpeedTimerCallback()
{
  std_msgs::msg::Float64 speed_msg;
  speed_msg.data = 0.0;
  for (const auto &region_config : regions_)
  {
    if (region_name_.find(region_config.name) == std::string::npos)
    {
      continue;
    }

    if (speed_level_ < region_config.speeds.size())
    {
      speed_msg.data = region_config.speeds[speed_level_];
    }
    else
    {
      speed_msg.data = region_config.speeds.back();
    }
    break;
  }
  speed_pub_->publish(speed_msg);
}
