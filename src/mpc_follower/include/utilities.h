// mpc_visualization.hpp
#ifndef MPC_FOLLOWER__MPC_VISUALIZATION_HPP_
#define MPC_FOLLOWER__MPC_VISUALIZATION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

// 前向声明，避免循环包含
namespace mpc_follower {
    struct RobotState { double x, y, theta, vx, vy, omega; };
    struct DynamicObstacle { double x, y, radius; };
}

namespace mpc_viz {

using RobotState = mpc_follower::RobotState;
using DynamicObstacle = mpc_follower::DynamicObstacle;

/**
 * @brief 把轨迹点数组封装成 nav_msgs::Path
 * @param trajectory 预测轨迹（世界坐标系）
 * @param frame_id 通常为 "map" 或 "odom"
 * @param stamp 时间戳
 * @param z_offset 可视化高度偏移（避免贴地），默认0.15m
 */
inline nav_msgs::msg::Path createPathMsg(
    const std::vector<RobotState>& trajectory,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    double z_offset = 0.15)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = stamp;
    path.poses.reserve(trajectory.size());

    for (const auto& s : trajectory) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = s.x;
        pose.pose.position.y = s.y;
        pose.pose.position.z = z_offset;

        tf2::Quaternion q;
        q.setRPY(0, 0, s.theta);
        pose.pose.orientation = tf2::toMsg(q);
        path.poses.push_back(pose);
    }
    return path;
}

/**
 * @brief 把障碍物列表封装成 MarkerArray（红色圆柱）
 * @param obstacles 动态障碍物容器
 * @param frame_id 参考系
 * @param stamp 时间戳
 * @param lifetime_sec 可视化持续时间（太短会闪烁，太长会拖尾）
 */
inline visualization_msgs::msg::MarkerArray createObstacleMarkers(
    const std::vector<DynamicObstacle>& obstacles,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    double lifetime_sec = 0.3)
{
    visualization_msgs::msg::MarkerArray ma;
    ma.markers.reserve(obstacles.size());

    for (size_t i = 0; i < obstacles.size(); ++i) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = stamp;
        m.ns = "mpc_obstacles";
        m.id = static_cast<int>(i);
        m.type = visualization_msgs::msg::Marker::CYLINDER;
        m.action = visualization_msgs::msg::Marker::ADD;
        
        m.pose.position.x = obstacles[i].x;
        m.pose.position.y = obstacles[i].y;
        m.pose.position.z = 0.5;  // 圆柱中心高度
        
        m.scale.x = obstacles[i].radius * 2.0;
        m.scale.y = obstacles[i].radius * 2.0;
        m.scale.z = 1.0;  // 圆柱高度
        
        m.color.r = 1.0;
        m.color.g = 0.2;
        m.color.b = 0.2;
        m.color.a = 0.6;
        
        m.lifetime = rclcpp::Duration::from_seconds(lifetime_sec);
        ma.markers.push_back(m);
    }
    return ma;
}

/**
 * @brief 清空指定id范围内的Marker（用于障碍物消失时擦除）
 * @param count 要清空的marker数量（通常传上一个周期的障碍物数）
 */
inline visualization_msgs::msg::MarkerArray createClearMarkers(
    size_t count,
    const std::string& frame_id,
    const rclcpp::Time& stamp)
{
    visualization_msgs::msg::MarkerArray ma;
    for (size_t i = 0; i < count; ++i) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = stamp;
        m.ns = "mpc_obstacles";
        m.id = static_cast<int>(i);
        m.action = visualization_msgs::msg::Marker::DELETE;
        ma.markers.push_back(m);
    }
    return ma;
}

}  // namespace mpc_viz

#endif  // MPC_FOLLOWER__MPC_VISUALIZATION_HPP_