#include <rclcpp/rclcpp.hpp>
#include "esdf_map.h"
#include "Astar.h"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <Eigen/Core>
#include <cmath>
#include "smoother.h"
#include "jps.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <algorithm>
#include <array>
#include <sstream>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <std_msgs/msg/bool.hpp>

class PlanManager : public rclcpp::Node
{
public:
  PlanManager()
  : Node("navi_planner"),
    map_geted(false),
    esdf_1(new ESDF_enviroment::esdf),
    planner_1(),
    smoother_1()
  {  
    // 初始化ROS2相关参数
    this->declare_parameter<bool>("enable_downstairs", false);
    this->declare_parameter<double>("obstacle_expand_radius", 0.40);
    // ROS1 legacy compatibility: old param name used in navigation_ws.
    this->declare_parameter<double>("voronoi_radius", -1.0);
    this->declare_parameter<double>("check_collision_radius", 0.50);
    this->declare_parameter<double>("obstacle_cost_weight", 0.0);
    this->declare_parameter<double>("dynamic_penalty_weight",100.0);
    // [DYN_GRID_REPLAN_V1] dynamic source + replan trigger params
    this->declare_parameter<bool>("use_pointcloud_dynamic_obstacles", true);
    this->declare_parameter<bool>("use_costmap_dynamic_obstacles", false);
    this->declare_parameter<bool>("use_costmap_clearance_fusion", true);
    this->declare_parameter<bool>("enable_blocked_path_truncation", true);
    this->declare_parameter<double>("blocked_path_truncation_margin", 0.20);
    this->declare_parameter<bool>("enable_blocked_zone_memory", true);
    this->declare_parameter<double>("blocked_zone_memory_radius", 0.45);
    this->declare_parameter<double>("blocked_zone_memory_ttl_sec", 2.0);
    this->declare_parameter<std::string>("costmap_topic", "/costmap/costmap");
    this->declare_parameter<std::string>("costmap_update_topic", "/costmap/costmap_updates");
    this->declare_parameter<int>("costmap_occupied_threshold", 50);
    this->declare_parameter<double>("monitor_period_sec", 0.2);
    this->declare_parameter<double>("min_replan_interval_sec", 0.5);
    this->declare_parameter<bool>("enable_stall_trigger", true);
    this->declare_parameter<double>("stall_window_sec", 8.0);
    this->declare_parameter<double>("stall_move_threshold", 0.05);
    this->declare_parameter<double>("stall_goal_distance_threshold", 0.30);
    this->declare_parameter<double>("robot_inscribed_radius", 0.25);
    this->declare_parameter<double>("clearance_margin", 0.05);
    this->declare_parameter<double>("forward_check_distance", 2.0);
    this->declare_parameter<double>("forward_sample_step", 0.10);
    this->declare_parameter<bool>("enable_start_escape_mode", true);
    this->declare_parameter<int>("start_escape_steps", 16);
    this->declare_parameter<bool>("enable_goal_escape_mode", true);
    this->declare_parameter<double>("goal_escape_search_radius", 0.80);
    this->declare_parameter<double>("goal_escape_clearance_tolerance", 0.08);
    // [JPS_V1] JPS frontend (report 5.5.3)
    this->declare_parameter<bool>("use_jps_frontend", false);
    this->declare_parameter<double>("jps_time_k1", 1.0);
    this->declare_parameter<double>("jps_time_k2", 0.3);
    this->declare_parameter<double>("jps_vmax", 1.8);
    this->declare_parameter<double>("jps_amax", 1.5);
    this->declare_parameter<double>("jps_dt", 0.1);
    this->declare_parameter<double>("jps_sample_ds", 0.05);

    this->get_parameter("enable_downstairs", enable_downstaris);
    this->get_parameter("obstacle_expand_radius", obstacle_expand_radius);
    double legacy_voronoi_radius = -1.0;
    this->get_parameter("voronoi_radius", legacy_voronoi_radius);
    if (legacy_voronoi_radius > 0.0) {
      obstacle_expand_radius = legacy_voronoi_radius;
    }
    this->get_parameter("check_collision_radius", collision_radius);
    this->get_parameter("obstacle_cost_weight", obstacle_cost_weight);
    this->get_parameter("dynamic_penalty_weight", dynamic_penalty_weight);
    // [DYN_GRID_REPLAN_V1] load dynamic/replan params
    this->get_parameter("use_pointcloud_dynamic_obstacles", use_pointcloud_dynamic_obstacles_);
    this->get_parameter("use_costmap_dynamic_obstacles", use_costmap_dynamic_obstacles_);
    this->get_parameter("use_costmap_clearance_fusion", use_costmap_clearance_fusion_);
    this->get_parameter("enable_blocked_path_truncation", enable_blocked_path_truncation_);
    this->get_parameter("blocked_path_truncation_margin", blocked_path_truncation_margin_);
    this->get_parameter("enable_blocked_zone_memory", enable_blocked_zone_memory_);
    this->get_parameter("blocked_zone_memory_radius", blocked_zone_memory_radius_);
    this->get_parameter("blocked_zone_memory_ttl_sec", blocked_zone_memory_ttl_sec_);
    this->get_parameter("costmap_topic", costmap_topic_);
    this->get_parameter("costmap_update_topic", costmap_update_topic_);
    this->get_parameter("costmap_occupied_threshold", costmap_occupied_threshold_);
    this->get_parameter("monitor_period_sec", monitor_period_sec_);
    this->get_parameter("min_replan_interval_sec", min_replan_interval_sec_);
    this->get_parameter("enable_stall_trigger", enable_stall_trigger_);
    this->get_parameter("stall_window_sec", stall_window_sec_);
    this->get_parameter("stall_move_threshold", stall_move_threshold_);
    this->get_parameter("stall_goal_distance_threshold", stall_goal_distance_threshold_);
    this->get_parameter("robot_inscribed_radius", robot_inscribed_radius_);
    this->get_parameter("clearance_margin", clearance_margin_);
    this->get_parameter("forward_check_distance", forward_check_distance_);
    this->get_parameter("forward_sample_step", forward_sample_step_);
    this->get_parameter("enable_start_escape_mode", enable_start_escape_mode_);
    this->get_parameter("start_escape_steps", start_escape_steps_);
    this->get_parameter("enable_goal_escape_mode", enable_goal_escape_mode_);
    this->get_parameter("goal_escape_search_radius", goal_escape_search_radius_);
    this->get_parameter("goal_escape_clearance_tolerance", goal_escape_clearance_tolerance_);
    this->get_parameter("use_jps_frontend", use_jps_frontend_);
    this->get_parameter("jps_time_k1", jps_time_k1_);
    this->get_parameter("jps_time_k2", jps_time_k2_);
    this->get_parameter("jps_vmax", jps_vmax_);
    this->get_parameter("jps_amax", jps_amax_);
    this->get_parameter("jps_dt", jps_dt_);
    this->get_parameter("jps_sample_ds", jps_sample_ds_);

    RCLCPP_INFO(this->get_logger(), "[Params] [enable_downstairs] : %s", enable_downstaris ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "[Params] [obstacle_expand_radius] : %f", obstacle_expand_radius);
    if (legacy_voronoi_radius > 0.0) {
      RCLCPP_INFO(this->get_logger(), "[Compat] use legacy param voronoi_radius: %f", legacy_voronoi_radius);
    }
    RCLCPP_INFO(this->get_logger(), "[Params] [check_collision_radius] : %f", collision_radius);
    RCLCPP_INFO(this->get_logger(), "[Params] [obstacle_cost_weight] : %f", obstacle_cost_weight);
    RCLCPP_INFO(this->get_logger(), "[Params] [use_pointcloud_dynamic_obstacles] : %s", use_pointcloud_dynamic_obstacles_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "[Params] [use_costmap_dynamic_obstacles] : %s", use_costmap_dynamic_obstacles_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "[Params] [use_costmap_clearance_fusion] : %s", use_costmap_clearance_fusion_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "[Params] [enable_blocked_path_truncation] : %s", enable_blocked_path_truncation_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "[Params] [blocked_path_truncation_margin] : %f", blocked_path_truncation_margin_);
    RCLCPP_INFO(this->get_logger(), "[Params] [enable_blocked_zone_memory] : %s", enable_blocked_zone_memory_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "[Params] [blocked_zone_memory_radius] : %f", blocked_zone_memory_radius_);
    RCLCPP_INFO(this->get_logger(), "[Params] [blocked_zone_memory_ttl_sec] : %f", blocked_zone_memory_ttl_sec_);
    RCLCPP_INFO(this->get_logger(), "[Params] [costmap_topic] : %s", costmap_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "[Params] [costmap_update_topic] : %s", costmap_update_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "[Params] [monitor_period_sec] : %f", monitor_period_sec_);
    RCLCPP_INFO(this->get_logger(), "[Params] [robot_inscribed_radius] : %f", robot_inscribed_radius_);
    RCLCPP_INFO(this->get_logger(), "[Params] [clearance_margin] : %f", clearance_margin_);
    RCLCPP_INFO(this->get_logger(), "[Params] [forward_check_distance] : %f", forward_check_distance_);
    RCLCPP_INFO(this->get_logger(), "[Params] [forward_sample_step] : %f", forward_sample_step_);
    RCLCPP_INFO(this->get_logger(), "[Params] [enable_start_escape_mode] : %s", enable_start_escape_mode_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "[Params] [start_escape_steps] : %d", start_escape_steps_);
    RCLCPP_INFO(this->get_logger(), "[Params] [enable_goal_escape_mode] : %s", enable_goal_escape_mode_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "[Params] [goal_escape_search_radius] : %f", goal_escape_search_radius_);
    RCLCPP_INFO(this->get_logger(), "[Params] [goal_escape_clearance_tolerance] : %f", goal_escape_clearance_tolerance_);
    RCLCPP_INFO(this->get_logger(), "[Params] [use_jps_frontend] : %s", use_jps_frontend_ ? "true" : "false");

    // map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    //   "/map", 1, std::bind(&PlanManager::map_callback, this, std::placeholders::_1));
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
  "/map", 
  rclcpp::QoS(1).reliable().transient_local(),
  std::bind(&PlanManager::map_callback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 1, std::bind(&PlanManager::goal_callback, this, std::placeholders::_1));
    obstacle_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_livox_obs", 1, std::bind(&PlanManager::setObstacle, this,std::placeholders::_1));
    cost_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&PlanManager::cost_map_callback,this,std::placeholders::_1));
    cost_map_update_sub_ = this->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
      costmap_update_topic_,
      rclcpp::QoS(10).reliable().transient_local(),
      std::bind(&PlanManager::cost_map_update_callback, this, std::placeholders::_1));
    
    //publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/sPath", 1);
    reachable_pub_ = this->create_publisher<std_msgs::msg::Bool>("/ly/navi/reachable", 1);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    start = Eigen::Vector2d::Zero();
    goal = Eigen::Vector2d::Zero();
    has_goal_ = false;
    //TODO
    is_first_goal_ = true;
    last_goal_ = Eigen::Vector2d::Zero();

    const double monitor_period = std::max(0.02, monitor_period_sec_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(monitor_period)),
      std::bind(&PlanManager::timer_callback, this));
    // [DYN_GRID_REPLAN_V1] initialize replan/stall state
    last_replan_time_ = this->now() - rclcpp::Duration::from_seconds(3600.0);
    stall_anchor_time_ = this->now();
    stall_anchor_pose_ = Eigen::Vector2d::Zero();
    stall_anchor_initialized_ = false;
  }

private:
  bool map_geted;
  Eigen::Vector2i map_size;
  Eigen::Vector2d map_offset;
  double resolution;
  ESDF_enviroment::Ptr esdf_1;
  navi_planner::Astar planner_1;
  navi_planner::smoother smoother_1;
  navi_planner::JPS jps_planner_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_map_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr cost_map_update_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reachable_pub_;
  Eigen::Vector2d start, goal;
  std::vector<Eigen::Vector2d> path_now;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  bool enable_downstaris;
  double obstacle_expand_radius, collision_radius;
  double obstacle_cost_weight;
  //新增的stvl动态障碍参数
  double dynamic_penalty_weight;
  bool has_goal_;
  const std::string global_frame_{"map"};
  const std::array<std::string, 2> robot_frame_candidates_{{"base_link", "baselink"}};
  // [DYN_GRID_REPLAN_V1] dynamic obstacle source and replan/stall params
  bool use_pointcloud_dynamic_obstacles_{true};
  bool use_costmap_dynamic_obstacles_{false};
  bool use_costmap_clearance_fusion_{true};
  bool enable_blocked_path_truncation_{true};
  double blocked_path_truncation_margin_{0.20};
  bool enable_blocked_zone_memory_{true};
  double blocked_zone_memory_radius_{0.45};
  double blocked_zone_memory_ttl_sec_{2.0};
  std::string costmap_topic_{"/costmap/costmap"};
  std::string costmap_update_topic_{"/costmap/costmap_updates"};
  int costmap_occupied_threshold_{50};
  double monitor_period_sec_{0.2};
  double min_replan_interval_sec_{0.5};
  bool enable_stall_trigger_{true};
  double stall_window_sec_{8.0};
  double stall_move_threshold_{0.05};
  double stall_goal_distance_threshold_{0.30};
  double robot_inscribed_radius_{0.25};
  double clearance_margin_{0.05};
  double forward_check_distance_{2.0};
  double forward_sample_step_{0.10};
  bool enable_start_escape_mode_{true};
  int start_escape_steps_{16};
  bool enable_goal_escape_mode_{true};
  double goal_escape_search_radius_{0.80};
  double goal_escape_clearance_tolerance_{0.08};
  bool use_jps_frontend_{false};
  double jps_time_k1_{1.0};
  double jps_time_k2_{0.3};
  double jps_vmax_{1.8};
  double jps_amax_{1.5};
  double jps_dt_{0.1};
  double jps_sample_ds_{0.05};
  std::vector<Eigen::Vector2i> dynamic_cells_from_pointcloud_;
  std::vector<Eigen::Vector2i> dynamic_cells_from_costmap_;
  std::vector<Eigen::Vector2i> dynamic_cells_from_blocked_memory_;
  std::unordered_set<uint64_t> dynamic_costmap_cell_keys_;
  std::unordered_map<uint64_t, rclcpp::Time> blocked_memory_cell_expire_time_;
  nav_msgs::msg::MapMetaData latest_costmap_info_;
  bool has_costmap_info_{false};
  size_t costmap_callback_count_{0};
  size_t costmap_update_callback_count_{0};
  rclcpp::Time last_replan_time_;
  rclcpp::Time stall_anchor_time_;
  Eigen::Vector2d stall_anchor_pose_;
  bool stall_anchor_initialized_{false};

  //TODO
  rclcpp::TimerBase::SharedPtr timer_;
  Eigen::Vector2d last_goal_;
  bool is_first_goal_;

  void rebuildBlockedMemoryCells()
  {
    dynamic_cells_from_blocked_memory_.clear();
    dynamic_cells_from_blocked_memory_.reserve(blocked_memory_cell_expire_time_.size());
    for (const auto &kv : blocked_memory_cell_expire_time_)
    {
      const int row = static_cast<int>(kv.first >> 32);
      const int col = static_cast<int>(kv.first & 0xffffffffu);
      dynamic_cells_from_blocked_memory_.emplace_back(row, col);
    }
  }

  bool pruneExpiredBlockedMemoryCells()
  {
    if (!enable_blocked_zone_memory_)
    {
      const bool had_any =
        !blocked_memory_cell_expire_time_.empty() || !dynamic_cells_from_blocked_memory_.empty();
      blocked_memory_cell_expire_time_.clear();
      dynamic_cells_from_blocked_memory_.clear();
      return had_any;
    }

    const auto now = this->now();
    bool changed = false;
    for (auto it = blocked_memory_cell_expire_time_.begin(); it != blocked_memory_cell_expire_time_.end();)
    {
      if (it->second <= now)
      {
        it = blocked_memory_cell_expire_time_.erase(it);
        changed = true;
      }
      else
      {
        ++it;
      }
    }
    if (changed)
    {
      rebuildBlockedMemoryCells();
    }
    return changed;
  }

  void addBlockedMemoryZone(const Eigen::Vector2i &blocked_idx, const char *reason)
  {
    if (!enable_blocked_zone_memory_ || blocked_zone_memory_ttl_sec_ <= 0.0)
    {
      return;
    }

    const int radius_cells = std::max(1, static_cast<int>(
      std::ceil(std::max(0.0, blocked_zone_memory_radius_) / resolution)));
    const int radius_sq = radius_cells * radius_cells;
    const auto expire_time = this->now() + rclcpp::Duration::from_seconds(blocked_zone_memory_ttl_sec_);
    int inserted_cells = 0;
    int extended_cells = 0;
    for (int dr = -radius_cells; dr <= radius_cells; ++dr)
    {
      for (int dc = -radius_cells; dc <= radius_cells; ++dc)
      {
        if (dr * dr + dc * dc > radius_sq)
        {
          continue;
        }
        const int row = blocked_idx[0] + dr;
        const int col = blocked_idx[1] + dc;
        if (row < 0 || col < 0 || row >= map_size[0] || col >= map_size[1])
        {
          continue;
        }

        const uint64_t key = packMapCellKey(Eigen::Vector2i(row, col));
        auto it = blocked_memory_cell_expire_time_.find(key);
        if (it == blocked_memory_cell_expire_time_.end())
        {
          blocked_memory_cell_expire_time_.emplace(key, expire_time);
          ++inserted_cells;
        }
        else if (it->second < expire_time)
        {
          it->second = expire_time;
          ++extended_cells;
        }
      }
    }
    rebuildBlockedMemoryCells();
    applyMergedDynamicObstacles();
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "[BLOCK_MEMORY] reason=%s center=(%d,%d) radius_cells=%d inserted=%d extended=%d active=%zu",
      reason,
      blocked_idx[0], blocked_idx[1], radius_cells, inserted_cells, extended_cells,
      blocked_memory_cell_expire_time_.size());
  }

  // [DYN_GRID_REPLAN_V1] merge dynamic obstacle sources and update Voronoi once
  void applyMergedDynamicObstacles()
  {
    std::vector<Eigen::Vector2i> merged;
    merged.reserve(
      dynamic_cells_from_pointcloud_.size() +
      dynamic_cells_from_costmap_.size() +
      dynamic_cells_from_blocked_memory_.size());
    merged.insert(merged.end(), dynamic_cells_from_pointcloud_.begin(), dynamic_cells_from_pointcloud_.end());
    merged.insert(merged.end(), dynamic_cells_from_costmap_.begin(), dynamic_cells_from_costmap_.end());
    merged.insert(merged.end(), dynamic_cells_from_blocked_memory_.begin(), dynamic_cells_from_blocked_memory_.end());

    esdf_1->setDynamicObstacles(merged);
  }

  // nav2 costmap may encode costs as 0..254 in int8[] (254 becomes -2).
  // This helper supports both standard occupancy (-1/0..100) and nav2 cost semantics.
  int normalizeCostmapCellToOcc100(int8_t raw_cell) const
  {
    if (raw_cell >= 0 && raw_cell <= 100)
    {
      return static_cast<int>(raw_cell);
    }

    const int cell_u8 = static_cast<int>(static_cast<uint8_t>(raw_cell));
    if (cell_u8 == 255)
    {
      return -1;
    }
    return static_cast<int>(std::round(static_cast<double>(cell_u8) * 100.0 / 254.0));
  }

  uint64_t packMapCellKey(const Eigen::Vector2i &map_idx) const
  {
    return
      (static_cast<uint64_t>(static_cast<uint32_t>(map_idx[0])) << 32) |
      static_cast<uint32_t>(map_idx[1]);
  }

  void rebuildCostmapCellsFromKeySet()
  {
    dynamic_cells_from_costmap_.clear();
    dynamic_cells_from_costmap_.reserve(dynamic_costmap_cell_keys_.size());
    for (const auto &key : dynamic_costmap_cell_keys_)
    {
      const int row = static_cast<int>(key >> 32);
      const int col = static_cast<int>(key & 0xffffffffu);
      dynamic_cells_from_costmap_.emplace_back(row, col);
    }
  }

  bool convertCostmapCellToStaticMapIndex(
    int costmap_row,
    int costmap_col,
    const nav_msgs::msg::MapMetaData &info,
    Eigen::Vector2i &map_idx_out)
  {
    Eigen::Vector2d world_pt;
    world_pt[0] = info.origin.position.x + (static_cast<double>(costmap_col) + 0.5) * static_cast<double>(info.resolution);
    world_pt[1] = info.origin.position.y + (static_cast<double>(costmap_row) + 0.5) * static_cast<double>(info.resolution);
    map_idx_out = Pos2index(world_pt);
    return
      map_idx_out[0] >= 0 && map_idx_out[1] >= 0 &&
      map_idx_out[0] < map_size[0] && map_idx_out[1] < map_size[1];
  }

  void consumeCostmapRegion(
    const nav_msgs::msg::MapMetaData &info,
    int start_row,
    int start_col,
    int region_width,
    int region_height,
    const std::vector<int8_t> &data,
    const std::string &frame_id,
    const char *source_tag)
  {
    if (!map_geted)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[COSTMAP->VORONOI] skip (%s): static map is not ready.",
        source_tag);
      return;
    }
    if (!use_costmap_dynamic_obstacles_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "[COSTMAP->VORONOI] skip (%s): use_costmap_dynamic_obstacles=false.",
        source_tag);
      return;
    }

    const int map_width = static_cast<int>(info.width);
    const int map_height = static_cast<int>(info.height);
    if (map_width <= 0 || map_height <= 0 || data.empty())
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[COSTMAP->VORONOI] invalid %s message: map_size=%dx%d data_size=%zu",
        source_tag, map_width, map_height, data.size());
      return;
    }
    if (start_row < 0 || start_col < 0 || region_width <= 0 || region_height <= 0)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[COSTMAP->VORONOI] invalid %s region: row=%d col=%d w=%d h=%d",
        source_tag, start_row, start_col, region_width, region_height);
      return;
    }
    if (start_row + region_height > map_height || start_col + region_width > map_width)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[COSTMAP->VORONOI] out-of-range %s region: row=%d col=%d w=%d h=%d map=%dx%d",
        source_tag, start_row, start_col, region_width, region_height, map_width, map_height);
      return;
    }

    const bool full_region =
      (start_row == 0 && start_col == 0 && region_width == map_width && region_height == map_height);
    if (full_region)
    {
      dynamic_costmap_cell_keys_.clear();
    }

    const int threshold = std::max(0, std::min(100, costmap_occupied_threshold_));
    int unknown_cells = 0;
    int occupied_cells = 0;
    int accepted_cells = 0;
    int out_of_map_cells = 0;
    int inserted_cells = 0;
    int removed_cells = 0;

    const int expected_cells = region_width * region_height;
    const int total_cells = std::min(static_cast<int>(data.size()), expected_cells);
    for (int idx = 0; idx < total_cells; ++idx)
    {
      const int local_row = idx / region_width;
      const int local_col = idx % region_width;
      const int row = start_row + local_row;
      const int col = start_col + local_col;

      const int occ = normalizeCostmapCellToOcc100(data[idx]);
      if (occ < 0)
      {
        ++unknown_cells;
      }
      if (occ >= threshold)
      {
        ++occupied_cells;
      }

      const bool should_insert = occ >= threshold;
      if (!should_insert && full_region)
      {
        continue;
      }

      Eigen::Vector2i map_idx;
      if (!convertCostmapCellToStaticMapIndex(row, col, info, map_idx))
      {
        ++out_of_map_cells;
        continue;
      }

      const uint64_t key = packMapCellKey(map_idx);
      if (should_insert)
      {
        ++accepted_cells;
        if (dynamic_costmap_cell_keys_.insert(key).second)
        {
          ++inserted_cells;
        }
      }
      else
      {
        if (dynamic_costmap_cell_keys_.erase(key) > 0)
        {
          ++removed_cells;
        }
      }
    }

    rebuildCostmapCellsFromKeySet();
    applyMergedDynamicObstacles();
    const size_t merged_cells =
      dynamic_cells_from_costmap_.size() +
      dynamic_cells_from_pointcloud_.size() +
      dynamic_cells_from_blocked_memory_.size();

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[COSTMAP->VORONOI][%s] frame=%s map=%dx%d res=%.3f origin=(%.2f,%.2f) region=(r%d,c%d,w%d,h%d) "
      "thr=%d occ=%d unknown=%d accepted=%d inserted=%d removed=%d out_of_map=%d dyn_costmap=%zu dyn_cloud=%zu dyn_block=%zu merged=%zu",
      source_tag,
      frame_id.c_str(),
      map_width, map_height, static_cast<double>(info.resolution), info.origin.position.x, info.origin.position.y,
      start_row, start_col, region_width, region_height,
      threshold,
      occupied_cells, unknown_cells, accepted_cells, inserted_cells, removed_cells, out_of_map_cells,
      dynamic_cells_from_costmap_.size(), dynamic_cells_from_pointcloud_.size(),
      dynamic_cells_from_blocked_memory_.size(), merged_cells);

    if (has_goal_)
    {
      if (!getStart(false))
      {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Skip costmap runtime check because current robot pose is unavailable.");
        return;
      }
      double min_clearance_m = std::numeric_limits<double>::infinity();
      if (!checkForwardPassability(min_clearance_m))
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Forward path not passable after costmap update: min_clearance=%.3f m required=%.3f m",
          min_clearance_m,
          requiredClearanceMeters());
        truncatePathOnBlockingObstacle("costmap forward not passable");
        triggerReplan("costmap forward not passable");
        return;
      }
      if (detectCollision())
      {
        truncatePathOnBlockingObstacle("costmap collision");
        triggerReplan("costmap collision");
      }
    }
  }

  // [DYN_GRID_REPLAN_V1] gate replan frequency and centralize replan entry
  void triggerReplan(const char *reason, bool reset_stall_anchor = false)
  {
    if (!has_goal_) return;
    if (!getStart(false))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Skip replanning because current robot pose is unavailable");
      return;
    }
    if ((start - goal).norm() < 0.1)
    {
      return;
    }

    const auto now = this->now();
    if ((now - last_replan_time_).seconds() < min_replan_interval_sec_)
    {
      return;
    }
    last_replan_time_ = now;
    RCLCPP_INFO(this->get_logger(), "Trigger replan: %s", reason);
    plan(start, goal);
    if (reset_stall_anchor)
    {
      // [DYN_GRID_REPLAN_V1] reset stall anchor only for selected triggers
      stall_anchor_pose_ = start;
      stall_anchor_time_ = now;
      stall_anchor_initialized_ = true;
    }
  }

  // [DYN_GRID_REPLAN_V1] stall detector: goal exists but robot hardly moved for a time window
  //检测机器人是否停滞，即在有路径情况下却并未移动，触发重规划
  bool detectStall()
  {
    if (!enable_stall_trigger_) return false;
    if (!has_goal_) return false;
    if ((start - goal).norm() < stall_goal_distance_threshold_) return false;

    const auto now = this->now();
    if (!stall_anchor_initialized_)
    {
      stall_anchor_pose_ = start;
      stall_anchor_time_ = now;
      stall_anchor_initialized_ = true;
      return false;
    }

    const double moved = (start - stall_anchor_pose_).norm();
    if (moved > stall_move_threshold_)
    {
      stall_anchor_pose_ = start;
      stall_anchor_time_ = now;
      return false;
    }

    return (now - stall_anchor_time_).seconds() >= stall_window_sec_;
  }
  // [修复]: 动态使用地图 resolution，不再硬编码除以 20.0
  Eigen::Vector2d Index2pos(Eigen::Vector2i index_)
  {
    Eigen::Vector2d pos;
    pos[0] = (double)index_[1] * resolution + map_offset[0];
    pos[1] = (double)index_[0] * resolution + map_offset[1];
    return pos;
  }

  // [修复]: 动态使用地图 resolution，并使用 std::floor 确保坐标轴负半区转换正确
  Eigen::Vector2i Pos2index(Eigen::Vector2d pos)
  {
    Eigen::Vector2i index;
    index[1] = (int)std::floor((pos[0] - map_offset[0]) / resolution);
    index[0] = (int)std::floor((pos[1] - map_offset[1]) / resolution);
    return index;
  }

  double requiredClearanceMeters() const
  {
    return std::max(0.0, robot_inscribed_radius_ + clearance_margin_);
  }

  double getCostmapClearanceMeters(
    const Eigen::Vector2i &map_idx,
    double required_clearance_m) const
  {
    if (!use_costmap_dynamic_obstacles_ || !use_costmap_clearance_fusion_ || dynamic_costmap_cell_keys_.empty())
    {
      return std::numeric_limits<double>::infinity();
    }

    const int key_row = map_idx[0];
    const int key_col = map_idx[1];
    const uint64_t center_key =
      (static_cast<uint64_t>(static_cast<uint32_t>(key_row)) << 32) |
      static_cast<uint32_t>(key_col);
    if (dynamic_costmap_cell_keys_.find(center_key) != dynamic_costmap_cell_keys_.end())
    {
      return 0.0;
    }

    const int radius_cells = std::max(1, static_cast<int>(std::ceil(required_clearance_m / resolution)));
    const int radius_sq = radius_cells * radius_cells;
    double min_dist_cells = std::numeric_limits<double>::infinity();

    for (int dr = -radius_cells; dr <= radius_cells; ++dr)
    {
      const int row = map_idx[0] + dr;
      if (row < 0 || row >= map_size[0])
      {
        continue;
      }
      for (int dc = -radius_cells; dc <= radius_cells; ++dc)
      {
        const int col = map_idx[1] + dc;
        if (col < 0 || col >= map_size[1])
        {
          continue;
        }
        const int dist_sq = dr * dr + dc * dc;
        if (dist_sq > radius_sq)
        {
          continue;
        }

        const uint64_t key =
          (static_cast<uint64_t>(static_cast<uint32_t>(row)) << 32) |
          static_cast<uint32_t>(col);
        if (dynamic_costmap_cell_keys_.find(key) == dynamic_costmap_cell_keys_.end())
        {
          continue;
        }

        const double dist_cells = std::sqrt(static_cast<double>(dist_sq));
        if (dist_cells < min_dist_cells)
        {
          min_dist_cells = dist_cells;
        }
      }
    }

    if (!std::isfinite(min_dist_cells))
    {
      return std::numeric_limits<double>::infinity();
    }
    return min_dist_cells * resolution;
  }

  bool getClearanceAtWorldPoint(
    const Eigen::Vector2d &world_pt,
    double &clearance_m,
    double costmap_search_radius_m = -1.0)
  {
    const Eigen::Vector2i idx = Pos2index(world_pt);
    if (
      idx[0] < 0 || idx[1] < 0 ||
      idx[0] >= map_size[0] || idx[1] >= map_size[1])
    {
      clearance_m = 0.0;
      return false;
    }

    const double dist_cells = esdf_1->getDist(idx);
    if (!std::isfinite(dist_cells))
    {
      clearance_m = 0.0;
      return false;
    }

    const double esdf_clearance_m = dist_cells * resolution;
    const double costmap_radius =
      (costmap_search_radius_m > 0.0) ? costmap_search_radius_m : requiredClearanceMeters();
    const double costmap_clearance_m = getCostmapClearanceMeters(idx, costmap_radius);
    clearance_m = std::min(esdf_clearance_m, costmap_clearance_m);
    return true;
  }

  void applyGoalEscapeIfNeeded(
    const Eigen::Vector2d &start_world,
    const Eigen::Vector2d &goal_world,
    Eigen::Vector2i &goal_index_io,
    Eigen::Vector2d &goal_world_used_io)
  {
    goal_world_used_io = goal_world;
    if (!enable_goal_escape_mode_ || !map_geted)
    {
      return;
    }

    const double required_clearance = requiredClearanceMeters();
    const double relaxed_clearance =
      std::max(0.0, required_clearance - std::max(0.0, goal_escape_clearance_tolerance_));

    double goal_clearance = 0.0;
    const bool goal_clearance_ok = getClearanceAtWorldPoint(goal_world, goal_clearance);
    const bool goal_collision = !goal_clearance_ok || esdf_1->checkCollision(goal_index_io);
    const bool goal_below_relaxed = !goal_clearance_ok || goal_clearance <= relaxed_clearance;
    if (!goal_collision && !goal_below_relaxed)
    {
      return;
    }

    const int radius_cells = std::max(1, static_cast<int>(std::ceil(goal_escape_search_radius_ / resolution)));
    const int radius_sq = radius_cells * radius_cells;

    auto select_candidate =
      [&](double clearance_threshold, Eigen::Vector2i &best_idx, double &best_clearance) -> bool
      {
        bool found = false;
        int best_goal_dist_sq = std::numeric_limits<int>::max();
        double best_start_dist_sq = std::numeric_limits<double>::infinity();
        best_clearance = 0.0;

        for (int dr = -radius_cells; dr <= radius_cells; ++dr)
        {
          for (int dc = -radius_cells; dc <= radius_cells; ++dc)
          {
            const int dist_sq = dr * dr + dc * dc;
            if (dist_sq > radius_sq)
            {
              continue;
            }

            Eigen::Vector2i idx(goal_index_io[0] + dr, goal_index_io[1] + dc);
            if (idx[0] < 0 || idx[1] < 0 || idx[0] >= map_size[0] || idx[1] >= map_size[1])
            {
              continue;
            }
            if (esdf_1->checkCollision(idx))
            {
              continue;
            }

            const Eigen::Vector2d cand_world = Index2pos(idx);
            double cand_clearance = 0.0;
            if (!getClearanceAtWorldPoint(cand_world, cand_clearance))
            {
              continue;
            }
            if (cand_clearance < clearance_threshold)
            {
              continue;
            }

            const double start_dist_sq = (cand_world - start_world).squaredNorm();
            const bool better =
              !found ||
              dist_sq < best_goal_dist_sq ||
              (dist_sq == best_goal_dist_sq && cand_clearance > best_clearance + 1e-6) ||
              (dist_sq == best_goal_dist_sq &&
               std::abs(cand_clearance - best_clearance) <= 1e-6 &&
               start_dist_sq < best_start_dist_sq);

            if (better)
            {
              found = true;
              best_goal_dist_sq = dist_sq;
              best_clearance = cand_clearance;
              best_start_dist_sq = start_dist_sq;
              best_idx = idx;
            }
          }
        }
        return found;
      };

    Eigen::Vector2i best_idx = goal_index_io;
    double best_clearance = 0.0;
    bool found = select_candidate(required_clearance, best_idx, best_clearance);
    if (!found)
    {
      found = select_candidate(relaxed_clearance, best_idx, best_clearance);
    }
    if (!found)
    {
      found = select_candidate(0.0, best_idx, best_clearance);
    }

    if (!found)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Goal escape failed: occupied/blocked goal at (%d,%d), radius=%.2f m, keep original goal.",
        goal_index_io[0], goal_index_io[1], goal_escape_search_radius_);
      return;
    }

    const Eigen::Vector2i old_idx = goal_index_io;
    goal_index_io = best_idx;
    goal_world_used_io = Index2pos(best_idx);
    RCLCPP_WARN(
      this->get_logger(),
      "Goal adjusted for escape: idx (%d,%d)->(%d,%d), clearance goal=%.3f used=%.3f required=%.3f relaxed=%.3f",
      old_idx[0], old_idx[1],
      best_idx[0], best_idx[1],
      goal_clearance,
      best_clearance,
      required_clearance,
      relaxed_clearance);
  }

  bool checkForwardPassability(double &min_clearance_m)
  {
    min_clearance_m = std::numeric_limits<double>::infinity();
    if (!map_geted || path_now.size() < 2 || forward_check_distance_ <= 0.0)
    {
      return true;
    }

    const double required_clearance = requiredClearanceMeters();
    if (required_clearance <= 0.0)
    {
      return true;
    }

    size_t nearest_idx = 0;
    double nearest_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < path_now.size(); ++i)
    {
      const double d = (path_now[i] - start).norm();
      if (d < nearest_dist)
      {
        nearest_dist = d;
        nearest_idx = i;
      }
    }

    const double sample_step = std::max(forward_sample_step_, resolution);
    double checked_distance = 0.0;
    bool sampled_any = false;

    auto sample_and_check = [&](const Eigen::Vector2d &pt) -> bool
    {
      double clearance_m = 0.0;
      if (!getClearanceAtWorldPoint(pt, clearance_m))
      {
        min_clearance_m = 0.0;
        return false;
      }
      sampled_any = true;
      min_clearance_m = std::min(min_clearance_m, clearance_m);
      return clearance_m > required_clearance;
    };

    if (!sample_and_check(start))
    {
      return false;
    }

    for (size_t i = nearest_idx; i + 1 < path_now.size(); ++i)
    {
      const Eigen::Vector2d seg_start = path_now[i];
      const Eigen::Vector2d seg_end = path_now[i + 1];
      const Eigen::Vector2d seg_vec = seg_end - seg_start;
      const double seg_len = seg_vec.norm();
      if (seg_len < 1e-6)
      {
        continue;
      }

      const double seg_budget = forward_check_distance_ - checked_distance;
      if (seg_budget <= 0.0)
      {
        break;
      }

      for (double s = 0.0; s <= seg_len; s += sample_step)
      {
        if (s > seg_budget)
        {
          break;
        }
        const Eigen::Vector2d pt = seg_start + seg_vec * (s / seg_len);
        if (!sample_and_check(pt))
        {
          return false;
        }
      }

      checked_distance += seg_len;
      if (checked_distance >= forward_check_distance_)
      {
        break;
      }
    }

    return sampled_any;
  }

  bool truncatePathOnBlockingObstacle(const char *reason)
  {
    if (!enable_blocked_path_truncation_ || !map_geted || path_now.size() < 2)
    {
      return false;
    }
    if (!getStart(false))
    {
      return false;
    }

    const double truncation_extra_margin_m = std::max(0.0, blocked_path_truncation_margin_);
    const double required_clearance = requiredClearanceMeters();
    const double safe_endpoint_clearance = required_clearance + truncation_extra_margin_m;

    size_t nearest_idx = 0;
    double nearest_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < path_now.size(); ++i)
    {
      const double d = (path_now[i] - start).norm();
      if (d < nearest_dist)
      {
        nearest_dist = d;
        nearest_idx = i;
      }
    }

    const double sample_step = std::max(forward_sample_step_, resolution);
    const double max_forward_distance =
      (forward_check_distance_ > 0.0) ? forward_check_distance_ : std::numeric_limits<double>::infinity();
    double checked_distance = 0.0;

    bool found_block = false;
    bool has_safe_endpoint = false;
    Eigen::Vector2d safe_endpoint = start;
    Eigen::Vector2d blocked_point = start;
    size_t safe_segment_index = nearest_idx;

    double start_clearance_m = 0.0;
    if (
      getClearanceAtWorldPoint(start, start_clearance_m, safe_endpoint_clearance) &&
      start_clearance_m > safe_endpoint_clearance)
    {
      has_safe_endpoint = true;
      safe_endpoint = start;
      safe_segment_index = nearest_idx;
    }

    for (size_t i = nearest_idx; i + 1 < path_now.size(); ++i)
    {
      const Eigen::Vector2d seg_start = path_now[i];
      const Eigen::Vector2d seg_end = path_now[i + 1];
      const Eigen::Vector2d seg_vec = seg_end - seg_start;
      const double seg_len = seg_vec.norm();
      if (seg_len < 1e-6)
      {
        continue;
      }

      const double seg_budget = max_forward_distance - checked_distance;
      if (seg_budget <= 0.0)
      {
        break;
      }
      const double seg_iter_limit = std::min(seg_len, seg_budget);

      for (double s = 0.0; s <= seg_iter_limit; s += sample_step)
      {
        const Eigen::Vector2d pt = seg_start + seg_vec * (s / seg_len);
        double clearance_required_m = 0.0;
        if (!getClearanceAtWorldPoint(pt, clearance_required_m, required_clearance))
        {
          clearance_required_m = 0.0;
        }

        if (clearance_required_m <= required_clearance)
        {
          found_block = true;
          blocked_point = pt;
          break;
        }

        double clearance_safe_m = clearance_required_m;
        if (safe_endpoint_clearance > required_clearance + 1e-6)
        {
          if (!getClearanceAtWorldPoint(pt, clearance_safe_m, safe_endpoint_clearance))
          {
            clearance_safe_m = 0.0;
          }
        }
        if (clearance_safe_m > safe_endpoint_clearance)
        {
          has_safe_endpoint = true;
          safe_endpoint = pt;
          safe_segment_index = i;
        }
      }

      if (found_block)
      {
        break;
      }
      checked_distance += seg_len;
      if (checked_distance >= max_forward_distance)
      {
        break;
      }
    }

    if (!found_block)
    {
      return false;
    }

    std::vector<Eigen::Vector2d> truncated_path;
    truncated_path.reserve(path_now.size());
    truncated_path.push_back(start);

    if (has_safe_endpoint)
    {
      for (size_t i = nearest_idx + 1; i <= safe_segment_index && i < path_now.size(); ++i)
      {
        if ((path_now[i] - truncated_path.back()).norm() > 1e-6)
        {
          truncated_path.push_back(path_now[i]);
        }
      }
      if ((safe_endpoint - truncated_path.back()).norm() > 1e-6)
      {
        truncated_path.push_back(safe_endpoint);
      }
    }

    if (truncated_path.size() < 2)
    {
      truncated_path.push_back(start);
    }

    path_now = truncated_path;
    Path_pub(path_now);

    double end_clearance_m = 0.0;
    if (!getClearanceAtWorldPoint(path_now.back(), end_clearance_m, safe_endpoint_clearance))
    {
      end_clearance_m = 0.0;
    }
    const Eigen::Vector2i blocked_idx = Pos2index(blocked_point);
    addBlockedMemoryZone(blocked_idx, reason);
    RCLCPP_WARN(
      this->get_logger(),
      "Path truncated before blocked cell for %s: blocked_idx=(%d,%d) end_clearance=%.3f required=%.3f hard_margin=%.3f",
      reason,
      blocked_idx[0], blocked_idx[1],
      end_clearance_m,
      required_clearance,
      truncation_extra_margin_m);
    return true;
  }

  void Path_pub(const std::vector<Eigen::Vector2d> &path2pub)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = this->now();

    for (auto p : path2pub)
    {
      geometry_msgs::msg::PoseStamped vertex;
      vertex.header = path.header;
      vertex.pose.position.x = p[0];
      vertex.pose.position.y = p[1];
      vertex.pose.orientation.w = 1.0;
      path.poses.push_back(vertex);
    }
    path_pub_->publish(path);
    RCLCPP_INFO(this->get_logger(), "Path has been published");
  }

  bool lookupLatestTransform(
    const std::string &target_frame,
    const std::string &source_frame,
    geometry_msgs::msg::TransformStamped &transform,
    bool verbose = true)
  {
    try
    {
      transform = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      if (verbose)
      {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "TF lookup failed (%s <- %s): %s",
          target_frame.c_str(),
          source_frame.c_str(),
          ex.what());
      }
      else
      {
        RCLCPP_DEBUG_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "TF lookup failed (%s <- %s): %s",
          target_frame.c_str(),
          source_frame.c_str(),
          ex.what());
      }
      return false;
    }
  }

  bool lookupLatestTransformEitherDirection(
    const std::string &target_frame,
    const std::string &source_frame,
    geometry_msgs::msg::TransformStamped &transform,
    bool verbose = true)
  {
    if (lookupLatestTransform(target_frame, source_frame, transform, verbose))
    {
      return true;
    }

    geometry_msgs::msg::TransformStamped inverse_transform;
    if (!lookupLatestTransform(source_frame, target_frame, inverse_transform, verbose))
    {
      return false;
    }

    Eigen::Isometry3d T_source_target = tf2::transformToEigen(inverse_transform.transform);
    Eigen::Isometry3d T_target_source = T_source_target.inverse();
    transform.header.stamp = inverse_transform.header.stamp;
    transform.header.frame_id = target_frame;
    transform.child_frame_id = source_frame;
    transform.transform = tf2::eigenToTransform(T_target_source).transform;
    return true;
  }

  bool poseToGlobal(
    const geometry_msgs::msg::PoseStamped &pose_in,
    Eigen::Vector2d &pose_xy_out,
    std::string *resolved_source_frame = nullptr)
  {
    const std::string source_frame = pose_in.header.frame_id.empty() ? global_frame_ : pose_in.header.frame_id;
    if (resolved_source_frame != nullptr)
    {
      *resolved_source_frame = source_frame;
    }

    geometry_msgs::msg::PoseStamped pose_latest = pose_in;
    pose_latest.header.frame_id = source_frame;
    pose_latest.header.stamp = builtin_interfaces::msg::Time();

    if (source_frame == global_frame_)
    {
      pose_xy_out[0] = pose_latest.pose.position.x;
      pose_xy_out[1] = pose_latest.pose.position.y;
      return true;
    }

    geometry_msgs::msg::TransformStamped transform;
    if (!lookupLatestTransform(global_frame_, source_frame, transform))
    {
      return false;
    }

    geometry_msgs::msg::PoseStamped pose_in_global;
    tf2::doTransform(pose_latest, pose_in_global, transform);
    pose_xy_out[0] = pose_in_global.pose.position.x;
    pose_xy_out[1] = pose_in_global.pose.position.y;
    return true;
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
  {
    if (map_geted) { return; }
    RCLCPP_INFO(this->get_logger(), "Map received");
    
    int size = map->info.width * map->info.height;
    map_size[0] = map->info.height;
    map_size[1] = map->info.width;
    map_offset[0] = map->info.origin.position.x;
    map_offset[1] = map->info.origin.position.y;
    resolution = map->info.resolution;
    
    std::cout << "map_size: " << map_size.transpose() << std::endl;
    std::cout << "map_offset: " << map_offset.transpose() << " resolution: " << resolution << std::endl;
    
    bool * bin_map_ = new bool[size];
    for (int i = 0; i < size; i++)
    {
      bin_map_[i] = map->data[i] ? true : false;
    }
    
    esdf_1->esdf_init(bin_map_, map->info.height, map->info.width, map_offset, enable_downstaris);
    
    // [修复]: 防止内存泄漏，初始化ESDF后释放临时分配的内存
    delete[] bin_map_; 
    
    RCLCPP_INFO(this->get_logger(), "ESDF map initialized");
    planner_1.setEnvironment(esdf_1);
    jps_planner_.setEnvironment(esdf_1);
    const double min_clearance_cells = requiredClearanceMeters() / resolution;
    planner_1.setParam(
      obstacle_cost_weight,
      dynamic_penalty_weight,
      min_clearance_cells,
      enable_start_escape_mode_,
      start_escape_steps_);
    planner_1.init();
    smoother_1.smoother_setEnvironment(esdf_1);
    
    map_geted = true;
  }

  void logNoPathDiagnostics(
    const char *stage,
    const Eigen::Vector2d &start_world,
    const Eigen::Vector2d &goal_world_raw,
    const Eigen::Vector2d &goal_world_used,
    const Eigen::Vector2i &start_idx,
    const Eigen::Vector2i &goal_idx)
  {
    double start_clearance_m = 0.0;
    double goal_clearance_m = 0.0;
    const bool start_clearance_ok = getClearanceAtWorldPoint(start_world, start_clearance_m);
    const bool goal_clearance_ok = getClearanceAtWorldPoint(goal_world_used, goal_clearance_m);
    const bool start_collision = esdf_1->checkCollision(start_idx);
    const bool goal_collision = esdf_1->checkCollision(goal_idx);
    const bool goal_adjusted = (goal_world_raw - goal_world_used).norm() > 1e-6;
    const std::string astar_debug = planner_1.formatLastSearchDebugInfo();

    RCLCPP_WARN(
      this->get_logger(),
      "[NO_PATH][%s] start=(%.3f,%.3f)->(%d,%d) goal_raw=(%.3f,%.3f) goal_used=(%.3f,%.3f)->(%d,%d) goal_adjusted=%s "
      "start_collision=%s goal_collision=%s start_clearance=%.3f(%s) goal_clearance=%.3f(%s) required_clearance=%.3f "
      "dynamic_cells[cloud=%zu costmap=%zu blocked=%zu] astar={%s}",
      stage,
      start_world[0], start_world[1], start_idx[0], start_idx[1],
      goal_world_raw[0], goal_world_raw[1],
      goal_world_used[0], goal_world_used[1], goal_idx[0], goal_idx[1],
      goal_adjusted ? "true" : "false",
      start_collision ? "true" : "false",
      goal_collision ? "true" : "false",
      start_clearance_m, start_clearance_ok ? "ok" : "invalid",
      goal_clearance_m, goal_clearance_ok ? "ok" : "invalid",
      requiredClearanceMeters(),
      dynamic_cells_from_pointcloud_.size(),
      dynamic_cells_from_costmap_.size(),
      dynamic_cells_from_blocked_memory_.size(),
      astar_debug.c_str());
  }

  // [JPS_V1] 起点被占据时，环形向外搜索最近自由栅格（替代 A* 的起点逃生）
  bool findNearestFreeCell(const Eigen::Vector2i &idx, int max_radius, Eigen::Vector2i &out)
  {
    if (!esdf_1->checkCollision(idx)) { out = idx; return true; }
    for (int r = 1; r <= max_radius; ++r)
    {
      for (int dr = -r; dr <= r; ++dr)
      {
        for (int dc = -r; dc <= r; ++dc)
        {
          if (std::max(std::abs(dr), std::abs(dc)) != r) continue;
          Eigen::Vector2i cand(idx[0] + dr, idx[1] + dc);
          if (cand[0] < 0 || cand[1] < 0 || cand[0] >= map_size[0] || cand[1] >= map_size[1]) continue;
          if (!esdf_1->checkCollision(cand)) { out = cand; return true; }
        }
      }
    }
    return false;
  }

  void plan(Eigen::Vector2d start_, Eigen::Vector2d goal_)
  {
    if (!map_geted)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "[NO_PATH][plan_skip_map_not_ready] map_geted=false start=(%.3f,%.3f) goal=(%.3f,%.3f)",
        start_[0], start_[1], goal_[0], goal_[1]);
      return;
    }

    // [修复]: 彻底清理多余的手动计算 sx, sy 逻辑，统一调用 Pos2index
    Eigen::Vector2i start_index = Pos2index(start_);
    Eigen::Vector2i end_index = Pos2index(goal_);
    Eigen::Vector2d planning_goal = goal_;

    // 越界保护
    if (start_index[0] < 0 || start_index[0] >= map_size[0] || 
        start_index[1] < 0 || start_index[1] >= map_size[1] ||
        end_index[0] < 0 || end_index[0] >= map_size[0] || 
        end_index[1] < 0 || end_index[1] >= map_size[1])
    {
	        RCLCPP_ERROR(this->get_logger(),
	                     "Goal/Start out of map bounds! sr=%d sc=%d gr=%d gc=%d  h=%d w=%d",
	                     start_index[0], start_index[1], end_index[0], end_index[1], map_size[0], map_size[1]);
        logNoPathDiagnostics(
          "input_out_of_map",
          start_,
          goal_,
          planning_goal,
          start_index,
          end_index);
		        return;
		    }

    applyGoalEscapeIfNeeded(start_, goal_, end_index, planning_goal);

		    RCLCPP_INFO(this->get_logger(), "Start planning...");

		    auto beforeTime = std::chrono::steady_clock::now();
    std::vector<Eigen::Vector2d> Path_2d;
    auto msg = std_msgs::msg::Bool();

    if (use_jps_frontend_)
    {
      // ===================== [JPS_V1] JPS 前端 + 时间分配（报告5.5.3 思路一） =====================
      bool jps_ok = jps_planner_.search(start_, planning_goal, Path_2d);
      if (!jps_ok && enable_start_escape_mode_ && esdf_1->checkCollision(start_index))
      {
        Eigen::Vector2i free_idx;
        if (findNearestFreeCell(start_index, std::max(start_escape_steps_, 40), free_idx))
        {
          const Eigen::Vector2d escaped_start = Index2pos(free_idx);
          jps_ok = jps_planner_.search(escaped_start, planning_goal, Path_2d);
          if (jps_ok && !Path_2d.empty())
          {
            Path_2d.insert(Path_2d.begin(), start_);
            RCLCPP_WARN(this->get_logger(), "[JPS] start occupied, escaped to cell (%d,%d)", free_idx[0], free_idx[1]);
          }
        }
      }
      if (!jps_ok)
      {
        logNoPathDiagnostics("jps_no_path", start_, goal_, planning_goal, start_index, end_index);
        msg.data = false;
        reachable_pub_->publish(msg);
        return;
      }
      msg.data = true;
      reachable_pub_->publish(msg);

      auto endTime = std::chrono::steady_clock::now();
      double duration_millsecond = std::chrono::duration<double, std::milli>(endTime - beforeTime).count();
      std::cout << "JPS searching cost: " << duration_millsecond << "ms" << std::endl;

      // 时间分配（思路一：梯形加减速）+ 等弧长密采样发 /sPath
      beforeTime = std::chrono::steady_clock::now();
      auto timed = navi_planner::allocateTimeTrapezoid(
        Path_2d, jps_time_k1_, jps_time_k2_, jps_vmax_, jps_amax_, jps_dt_);
      std::vector<Eigen::Vector2d> timed_pos;
      timed_pos.reserve(timed.size());
      for (const auto &tp : timed) timed_pos.push_back(tp.pos);
      Path_2d = navi_planner::resampleByArcLength(timed_pos, jps_sample_ds_);
      if (Path_2d.empty())
      {
        RCLCPP_WARN(this->get_logger(), "[JPS] resampled path empty, fallback to timed waypoints");
        Path_2d = timed_pos;
      }
      if (!Path_2d.empty())
      {
        Path_2d.front() = start_;
        Path_2d.back() = planning_goal;
      }
      endTime = std::chrono::steady_clock::now();
      duration_millsecond = std::chrono::duration<double, std::milli>(endTime - beforeTime).count();
      std::cout << "JPS time-alloc + resample cost: " << duration_millsecond << "ms" << std::endl;
    }
    else
    {
		    auto result = planner_1.search(start_index, end_index);
    bool retried_start_escape = false;
    const bool start_in_collision = esdf_1->checkCollision(start_index);
    if (result == navi_planner::Astar::NO_PATH && enable_start_escape_mode_ && start_in_collision)
    {
      logNoPathDiagnostics(
        "astar_no_path_first_try",
        start_,
        goal_,
        planning_goal,
        start_index,
        end_index);
      const int retry_escape_steps = std::max(start_escape_steps_, 40);
      if (retry_escape_steps > start_escape_steps_)
      {
        retried_start_escape = true;
        RCLCPP_WARN(
          this->get_logger(),
          "A* NO_PATH with occupied start. Retry once with wider escape window: %d -> %d steps",
          start_escape_steps_,
          retry_escape_steps);
        planner_1.reset();
        const double min_clearance_cells = requiredClearanceMeters() / resolution;
        planner_1.setParam(
          obstacle_cost_weight,
          dynamic_penalty_weight,
          min_clearance_cells,
          enable_start_escape_mode_,
          retry_escape_steps);
        result = planner_1.search(start_index, end_index);
        planner_1.setParam(
          obstacle_cost_weight,
          dynamic_penalty_weight,
          min_clearance_cells,
          enable_start_escape_mode_,
          start_escape_steps_);
      }
    }

    if (result == navi_planner::Astar::NO_PATH)
    {
      logNoPathDiagnostics(
        retried_start_escape ? "astar_no_path_after_retry" : "astar_no_path",
        start_,
        goal_,
        planning_goal,
        start_index,
        end_index);
      planner_1.reset();
      msg.data = false;
      reachable_pub_->publish(msg);
      return;
    }
    else
    {
        msg.data = true;
        reachable_pub_->publish(msg);
    }
    
    auto endTime = std::chrono::steady_clock::now();
    double duration_millsecond = std::chrono::duration<double, std::milli>(endTime - beforeTime).count();
    std::cout << "A* searching cost: " << duration_millsecond << "ms" << std::endl;
    
    std::vector<Eigen::Vector2i> Path_2i = planner_1.getPath();

    Path_2d.reserve(Path_2i.size() + 2);
    Path_2d.push_back(start_);
    for (auto p : Path_2i)
    {
      Eigen::Vector2d path_point = Index2pos(p);
      if ((path_point - Path_2d.back()).norm() < 1e-6)
      {
        continue;
      }
      Path_2d.push_back(path_point);
    }
    if ((planning_goal - Path_2d.back()).norm() >= 1e-6)
    {
      Path_2d.push_back(planning_goal);
    }

    beforeTime = std::chrono::steady_clock::now();
    std::vector<Eigen::Vector2d> raw_path = Path_2d;
    Path_2d = smoother_1.smooth(Path_2d, 0.3f, 0.04f);
    endTime = std::chrono::steady_clock::now();
    duration_millsecond = std::chrono::duration<double, std::milli>(endTime - beforeTime).count();
    std::cout << "Smooth cost: " << duration_millsecond << "ms" << std::endl;

    if (Path_2d.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Smoothed path is empty, fallback to raw path");
      Path_2d = std::move(raw_path);
    }

    Path_2d.front() = start_;
    if ((Path_2d.back() - planning_goal).norm() < 1e-6)
    {
      Path_2d.back() = planning_goal;
    }
    else
    {
      Path_2d.push_back(planning_goal);
    }
    }  // end else: A* + smoother 分支

    planner_1.reset();
    path_now = Path_2d;
    Path_pub(Path_2d);
  }

  bool getStart(bool verbose = true)
  {
    std::ostringstream tried_frames;
    for (const auto &robot_frame : robot_frame_candidates_)
    {
      geometry_msgs::msg::TransformStamped robot_global_pose;
      if (!lookupLatestTransform(global_frame_, robot_frame, robot_global_pose, verbose))
      {
        tried_frames << robot_frame << " ";
        continue;
      }

      double pose_x = robot_global_pose.transform.translation.x;
      double pose_y = robot_global_pose.transform.translation.y;
      start = Eigen::Vector2d(pose_x, pose_y);
      if (verbose)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Plan start resolved from frame '%s' at (%.3f, %.3f)",
          robot_frame.c_str(),
          pose_x,
          pose_y);
      }
      return true;
    }

    const std::array<std::string, 2> anchor_frame_candidates{{"odom", "lidar"}};
    for (const auto &robot_frame : robot_frame_candidates_)
    {
      for (const auto &anchor_frame : anchor_frame_candidates)
      {
        geometry_msgs::msg::TransformStamped map_to_anchor;
        geometry_msgs::msg::TransformStamped anchor_to_robot;
        if (!lookupLatestTransform(global_frame_, anchor_frame, map_to_anchor, verbose))
        {
          tried_frames << global_frame_ << "<-" << anchor_frame << " ";
          continue;
        }
        if (!lookupLatestTransformEitherDirection(anchor_frame, robot_frame, anchor_to_robot, verbose))
        {
          tried_frames << anchor_frame << "<->" << robot_frame << " ";
          continue;
        }

        Eigen::Isometry3d T_map_anchor = tf2::transformToEigen(map_to_anchor.transform);
        Eigen::Isometry3d T_anchor_robot = tf2::transformToEigen(anchor_to_robot.transform);
        Eigen::Isometry3d T_map_robot = T_map_anchor * T_anchor_robot;
        start = T_map_robot.translation().head<2>();
        if (verbose)
        {
          RCLCPP_INFO(
            this->get_logger(),
            "Plan start resolved from frame '%s' via anchor '%s' at (%.3f, %.3f)",
            robot_frame.c_str(),
            anchor_frame.c_str(),
            start[0],
            start[1]);
        }
        return true;
      }
    }

    if (verbose)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Unable to resolve robot start pose in %s. Tried frames: %s",
        global_frame_.c_str(),
        tried_frames.str().c_str());
    }
    else
    {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Unable to resolve robot start pose in %s. Tried frames: %s",
        global_frame_.c_str(),
        tried_frames.str().c_str());
    }
    return false;
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr end)
  {
    if (!map_geted) {
      RCLCPP_WARN(this->get_logger(), "Map not received yet!");
      return;
    }

    Eigen::Vector2d goal_pt;
    std::string goal_source_frame;
    if (!poseToGlobal(*end, goal_pt, &goal_source_frame))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to transform goal_pose into %s", global_frame_.c_str());
      return;
    }

    this->goal = goal_pt;
    has_goal_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Goal received in frame '%s', planned goal in %s: (%.3f, %.3f)",
      goal_source_frame.c_str(),
      global_frame_.c_str(),
      goal[0],
      goal[1]);

    double dist_to_last_goal = (goal_pt - last_goal_).norm();
    //TODO
    if (is_first_goal_ || dist_to_last_goal > 0.05) 
    {
      RCLCPP_INFO(this->get_logger(), "新目标点到达！立即触发规划。距离偏差: %f", dist_to_last_goal);
      last_goal_ = goal_pt;
      is_first_goal_ = false;

      if (!getStart()) {
        RCLCPP_WARN(this->get_logger(), "无法获取当前位姿，跳过规划");
        return;
      }
      
      if ((start - goal).norm() < 0.2) {
        RCLCPP_INFO(this->get_logger(), "目标点离起点太近，忽略。");
        return;
      }

      // 立即规划
      plan(start, goal);
      // [DYN_GRID_REPLAN_V1] reset stall anchor on new-goal replan
      stall_anchor_initialized_ = false;

      // 更新上一次的目标点和标志位
      
    }
    else 
    {
      // 目标点没变，直接 return，不阻塞回调！交由定时器处理
      // RCLCPP_DEBUG(this->get_logger(), "目标点未改变，交由定时器处理。");
      return; 
    }
    //BAK
    // if (!getStart())
    // {
    //   RCLCPP_WARN(this->get_logger(), "Skip planning because current robot pose is unavailable");
    //   return;
    // }

    // if ((start - goal).norm() < 0.3) {
    //   RCLCPP_INFO(this->get_logger(), "Goal too close to start, ignoring.");
    //   return;
    // }
    
    // plan(start, goal);
  }
  void cost_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    ++costmap_callback_count_;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "[COSTMAP->VORONOI] callback hit: count=%zu frame=%s map_ready=%s use_costmap=%s",
      costmap_callback_count_,
      msg->header.frame_id.c_str(),
      map_geted ? "true" : "false",
      use_costmap_dynamic_obstacles_ ? "true" : "false");

    latest_costmap_info_ = msg->info;
    has_costmap_info_ = true;

    consumeCostmapRegion(
      msg->info,
      0,
      0,
      static_cast<int>(msg->info.width),
      static_cast<int>(msg->info.height),
      msg->data,
      msg->header.frame_id,
      "full");
  }

  void cost_map_update_callback(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg)
  {
    ++costmap_update_callback_count_;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "[COSTMAP->VORONOI] update callback hit: count=%zu frame=%s map_ready=%s have_full=%s use_costmap=%s",
      costmap_update_callback_count_,
      msg->header.frame_id.c_str(),
      map_geted ? "true" : "false",
      has_costmap_info_ ? "true" : "false",
      use_costmap_dynamic_obstacles_ ? "true" : "false");

    if (!has_costmap_info_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[COSTMAP->VORONOI] skip update: waiting for full map snapshot on %s.",
        costmap_topic_.c_str());
      return;
    }

    consumeCostmapRegion(
      latest_costmap_info_,
      static_cast<int>(msg->y),
      static_cast<int>(msg->x),
      static_cast<int>(msg->width),
      static_cast<int>(msg->height),
      msg->data,
      msg->header.frame_id,
      "update");
  }

  bool detectCollision()
  {
    const double required_clearance =
      std::max(collision_radius, requiredClearanceMeters());

    for (size_t i = 1; i < path_now.size(); i++)
    {
      Eigen::Vector2i path_node_index = Pos2index(path_now[i]);
      if (
        path_node_index[0] < 0 || path_node_index[1] < 0 ||
        path_node_index[0] >= map_size[0] || path_node_index[1] >= map_size[1])
      {
        RCLCPP_INFO(this->get_logger(), "Collision detected (direct)");
        return true;
      }

      const double dist_cells = esdf_1->getDist(path_node_index);
      if (!std::isfinite(dist_cells))
      {
        RCLCPP_INFO(this->get_logger(), "Collision detected (invalid dist)");
        return true;
      }

      const double dist_m = dist_cells * resolution;
      if (dist_m <= required_clearance)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Collision detected (clearance): dist=%.3f m required=%.3f m",
          dist_m,
          required_clearance);
        return true;
      }
    }
    return false;
  }

  void timer_callback()
  {
    if (map_geted && pruneExpiredBlockedMemoryCells())
    {
      applyMergedDynamicObstacles();
    }

    // 如果还没接到过目标，直接跳过
    if (!has_goal_) {
      return;
    }

    // 获取当前最新位姿
    if (!getStart(false)) {
      return;
    }

    // 防抖：如果你离目标已经很近了，就不必一直规划了
    if ((start - goal).norm() < 0.1) {
      return;
    }
    double min_clearance_m = std::numeric_limits<double>::infinity();
    if (!checkForwardPassability(min_clearance_m))
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Forward path not passable: min_clearance=%.3f m required=%.3f m",
        min_clearance_m,
        requiredClearanceMeters());
      truncatePathOnBlockingObstacle("timer forward not passable");
      triggerReplan("forward not passable");
      return;
    }
    if (detectCollision()) {
      // [DYN_GRID_REPLAN_V1] collision trigger from periodic timer
      truncatePathOnBlockingObstacle("timer collision");
      triggerReplan("timer collision");
      return;
    }
    if (detectStall())
    {
      // [DYN_GRID_REPLAN_V1] stall trigger from periodic timer
      triggerReplan("stall", true);
      return;
    }

    triggerReplan("timer periodic");
  }

  void setObstacle(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    if (!map_geted) return;
    if (!use_pointcloud_dynamic_obstacles_) return;
    pcl::PointCloud<pcl::PointXYZ> buffer;
    pcl::fromROSMsg(*cloud_msg, buffer);

    const std::string cloud_frame = cloud_msg->header.frame_id.empty() ? global_frame_ : cloud_msg->header.frame_id;
    if (cloud_frame != global_frame_)
    {
      geometry_msgs::msg::TransformStamped transform;
      if (!lookupLatestTransform(global_frame_, cloud_frame, transform))
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Skip obstacle update because cloud frame '%s' cannot be transformed to %s",
          cloud_frame.c_str(),
          global_frame_.c_str());
        return;
      }

      Eigen::Matrix4f tf_matrix = tf2::transformToEigen(transform.transform).matrix().cast<float>();
      pcl::PointCloud<pcl::PointXYZ> transformed_buffer;
      pcl::transformPointCloud(buffer, transformed_buffer, tf_matrix);
      buffer.swap(transformed_buffer);
    }

    // 动态计算膨胀半径的像素格数
    int expand_cells = (int)std::round(obstacle_expand_radius / resolution);
    std::vector<Eigen::Vector2i> dynamic_cells;
    dynamic_cells.reserve(buffer.points.size() * ((2 * expand_cells + 1) * (2 * expand_cells + 1)));
    
    for (auto &p : buffer.points)
    {
      Eigen::Vector2d obs_pt(p.x, p.y);
      Eigen::Vector2i obs_index = Pos2index(obs_pt);
      
      for (int k1 = -expand_cells; k1 <= expand_cells; k1++)
      {
        for (int k2 = -expand_cells; k2 <= expand_cells; k2++)
        {
          if (k1 * k1 + k2 * k2 > expand_cells * expand_cells) continue;
          int ix = obs_index[0] + k1;
          int iy = obs_index[1] + k2;
          if (ix < 0 || iy < 0 || ix >= esdf_1->Size[0] || iy >= esdf_1->Size[1]) continue;
          dynamic_cells.emplace_back(ix, iy);
        }
      }
    }

    // [DYN_GRID_REPLAN_V1] keep pointcloud source local, then merge with other dynamic sources
    dynamic_cells_from_pointcloud_.swap(dynamic_cells);
    applyMergedDynamicObstacles();

    if (!has_goal_)
    {
      return;
    }
    if (!getStart(false))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Skip pointcloud runtime check because current robot pose is unavailable.");
      return;
    }
    double min_clearance_m = std::numeric_limits<double>::infinity();
    if (!checkForwardPassability(min_clearance_m))
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Forward path not passable after pointcloud update: min_clearance=%.3f m required=%.3f m",
        min_clearance_m,
        requiredClearanceMeters());
      truncatePathOnBlockingObstacle("pointcloud forward not passable");
      triggerReplan("pointcloud forward not passable");
      return;
    }
    if (detectCollision())
    {
      // [DYN_GRID_REPLAN_V1] collision trigger from pointcloud obstacle update
      truncatePathOnBlockingObstacle("pointcloud collision");
      triggerReplan("pointcloud collision");
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
