#include "region_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <unordered_set>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

RegionDetector::RegionDetector()
: Node("region_detector"),
  tf_buffer_(this->get_clock())
{
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/sPath",
    1,
    std::bind(&RegionDetector::pathCallback, this, std::placeholders::_1));
  speed_level_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
    "/ly/navi/speed_level",
    1,
    std::bind(&RegionDetector::speedLevelCallback, this, std::placeholders::_1));

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 1);
  directional_region_pub_ =
    this->create_publisher<std_msgs::msg::String>("/ly/navi/directional_region", 1);

  is_rotated_pub_ = this->create_publisher<std_msgs::msg::Bool>("/ly/navi/should_rotate", 1);

  std::string default_config;
  try
  {
    default_config =
      ament_index_cpp::get_package_share_directory("region_detector") + "/cfg/region.yaml";
  }
  catch (const std::exception &e)
  {
    RCLCPP_WARN(this->get_logger(), "Failed to locate package share: %s", e.what());
  }
  this->declare_parameter<std::string>("config_file", default_config);
  std::string config_file = this->get_parameter("config_file").as_string();
  if (!loadFromYaml(config_file))
  {
    RCLCPP_WARN(this->get_logger(), "Region config not loaded, detector will publish default.");
  }

  pose_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&RegionDetector::poseTimerCallback, this));
}

bool RegionDetector::loadFromYaml(const std::string &config_file)
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
    if (!node["name"] || !node["region"] || !node["direction"])
    {
      RCLCPP_WARN(this->get_logger(), "Skip malformed region config entry.");
      continue;
    }

    RegionConfig region;
    region.name = node["name"].as<std::string>();

    const auto polygon_node = node["region"];
    if (!polygon_node.IsSequence())
    {
      RCLCPP_WARN(this->get_logger(), "Region %s has invalid polygon.", region.name.c_str());
      continue;
    }

    for (const auto &pt_node : polygon_node)
    {
      if (!pt_node.IsSequence() || pt_node.size() < 2)
      {
        continue;
      }
      geometry_msgs::msg::Point p;
      p.x = pt_node[0].as<double>();
      p.y = pt_node[1].as<double>();
      p.z = 0.0;
      region.polygon.push_back(p);
    }

    const auto direction_node = node["direction"];
    if (!direction_node.IsSequence() || direction_node.size() != 2 ||
      !direction_node[0].IsSequence() || !direction_node[1].IsSequence() ||
      direction_node[0].size() < 2 || direction_node[1].size() < 2)
    {
      RCLCPP_WARN(this->get_logger(), "Region %s has invalid direction.", region.name.c_str());
      continue;
    }

    region.direction_start.x = direction_node[0][0].as<double>();
    region.direction_start.y = direction_node[0][1].as<double>();
    region.direction_start.z = 0.0;
    region.direction_end.x = direction_node[1][0].as<double>();
    region.direction_end.y = direction_node[1][1].as<double>();
    region.direction_end.z = 0.0;

    if (region.polygon.size() >= 3)
    {
      regions_.push_back(region);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Region %s has less than 3 polygon points, ignored.", region.name.c_str());
    }
  }

  RCLCPP_INFO(this->get_logger(), "Loaded %zu region configs from %s", regions_.size(), config_file.c_str());
  return !regions_.empty();
}

void RegionDetector::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  path_points_.clear();
  path_points_.reserve(msg->poses.size());
  for (const auto &pose : msg->poses)
  {
    path_points_.push_back(pose.pose.position);
  }
}

void RegionDetector::poseTimerCallback()
{
  geometry_msgs::msg::TransformStamped robot_global_pose;
  try
  {
    robot_global_pose = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    current_position_.x = robot_global_pose.transform.translation.x;
    current_position_.y = robot_global_pose.transform.translation.y;
    current_position_.z = 0.0;
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return;
  }
  checkLogic();
  checkRotation();
}

void RegionDetector::speedLevelCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data <= 2)
  {
    speed_level_ = msg->data;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid speed level: %u. Must be 0, 1, or 2.", msg->data);
  }
}

bool RegionDetector::isPointInPolygon(
  const geometry_msgs::msg::Point &pt,
  const std::vector<geometry_msgs::msg::Point> &poly) const
{
  int count = 0;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
  {
    if (((poly[i].y > pt.y) != (poly[j].y > pt.y)) &&
      (pt.x < (poly[j].x - poly[i].x) * (pt.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x))
    {
      count++;
    }
  }
  return (count % 2) == 1;
}

void RegionDetector::checkLogic()
{
  bool any_active = false;
  std_msgs::msg::String region_msg;
  region_msg.data = "default";

  for (auto &region : regions_)
  {
    region.active = false;
    if (!isPointInPolygon(current_position_, region.polygon))
    {
      continue;
    }

    if (path_points_.empty())
    {
      continue;
    }

    geometry_msgs::msg::Point last_point = path_points_.back();
    for (int i = static_cast<int>(path_points_.size()) - 1; i >= 0; --i)
    {
      if (!isPointInPolygon(path_points_[static_cast<size_t>(i)], region.polygon))
      {
        last_point = path_points_[static_cast<size_t>(i)];
        break;
      }
    }

    geometry_msgs::msg::Point vec_path;
    vec_path.x = last_point.x - current_position_.x;
    vec_path.y = last_point.y - current_position_.y;

    geometry_msgs::msg::Point vec_ref;
    vec_ref.x = region.direction_end.x - region.direction_start.x;
    vec_ref.y = region.direction_end.y - region.direction_start.y;

    const double dot = vec_path.x * vec_ref.x + vec_path.y * vec_ref.y;
    const double mag_path = std::hypot(vec_path.x, vec_path.y);
    const double mag_ref = std::hypot(vec_ref.x, vec_ref.y);
    if (mag_path < 1e-3 || mag_ref < 1e-3)
    {
      continue;
    }

    double cos_theta = dot / (mag_path * mag_ref);
    cos_theta = std::clamp(cos_theta, -1.0, 1.0);
    constexpr double kRadToDeg = 57.2957795130823208768;
    const double angle = std::acos(cos_theta) * kRadToDeg;
    if (angle <= 45.0)
    {
      region.active = true;
      any_active = true;
      region_msg.data = region.name;
    }
  }

  if (!any_active)
  {
    region_msg.data = "default";
  }
  directional_region_pub_->publish(region_msg);
  publishMarkers();
}

void RegionDetector::checkRotation()
{
  static const std::unordered_set<std::string> kNoRotateRegionNames = {
    "blue_castle",
    "blue_left_hill",
    "red_castle",
    "red_left_hill"
  };

  std_msgs::msg::Bool is_rotated_msg;
  is_rotated_msg.data = true;
  for (const auto &region : regions_)
  {
    if (kNoRotateRegionNames.find(region.name) == kNoRotateRegionNames.end())
    {
      continue;
    }
    if (isPointInPolygon(current_position_, region.polygon))
    {
      is_rotated_msg.data = false;
      break;
    }
  }
  is_rotated_pub_->publish(is_rotated_msg);
}

void RegionDetector::publishMarkers()
{
  int id = 0;
  for (const auto &region : regions_)
  {
    visualization_msgs::msg::Marker poly;
    poly.header.frame_id = "map";
    poly.header.stamp = this->now();
    poly.ns = region.name;
    poly.id = id++;
    poly.type = visualization_msgs::msg::Marker::LINE_STRIP;
    poly.action = visualization_msgs::msg::Marker::ADD;
    poly.scale.x = 0.1;
    poly.color.r = region.active ? 1.0 : 0.0;
    poly.color.g = 0.0;
    poly.color.b = region.active ? 0.0 : 1.0;
    poly.color.a = region.active ? 1.0 : 0.2;

    poly.points = region.polygon;
    poly.points.push_back(region.polygon.front());
    marker_pub_->publish(poly);

    visualization_msgs::msg::Marker text;
    text.header = poly.header;
    text.ns = region.name + "_text";
    text.id = id++;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point p = region.polygon.front();
    p.z = 0.5;
    text.pose.position = p;
    text.scale.z = 0.5;
    text.color.a = 1.0;
    text.color.r = 1.0;
    text.text = (region.active ? "[active]" : "") + region.name;
    marker_pub_->publish(text);

    visualization_msgs::msg::Marker dir_marker;
    dir_marker.header = poly.header;
    dir_marker.ns = region.name + "_direction";
    dir_marker.id = id++;
    dir_marker.type = visualization_msgs::msg::Marker::ARROW;
    dir_marker.action = visualization_msgs::msg::Marker::ADD;
    dir_marker.scale.x = 0.1;
    dir_marker.scale.y = 0.1;
    dir_marker.scale.z = 0.1;
    dir_marker.color.r = 1.0;
    dir_marker.color.a = 0.8;
    dir_marker.points = {region.direction_start, region.direction_end};
    marker_pub_->publish(dir_marker);
  }
}
