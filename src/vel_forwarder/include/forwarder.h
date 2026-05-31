#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gimbal_driver/msg/vel.hpp>

namespace vel_forwarder
{
    class Forwarder : public rclcpp::Node
    {
    public:
        Forwarder();
        
    private:
        void cmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
        // 改成正确的类型
        rclcpp::Publisher<gimbal_driver::msg::Vel>::SharedPtr pub_ly_vel;
    };
} // namespace vel_forwarder