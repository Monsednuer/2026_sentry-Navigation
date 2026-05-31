// #include "forwarder.h"

// namespace vel_forwarder
// {
//     constexpr float MaxSpeed = 2.0f;
//     constexpr char MaxSpeedLevel = 70;
//     constexpr float SpeedRatio = MaxSpeedLevel / MaxSpeed;

//     char speed_mapping(float vel_in)
//     {
//         if (vel_in > 10.0f)
//         {
//             RCLCPP_WARN(rclcpp::get_logger("forwarder"), "Bad speed input: %.2f", vel_in);
//             return 0;
//         }
//         vel_in = vel_in > 2.4f ? 2.4f : (vel_in < -2.4f ? -2.4f : vel_in);
//         return static_cast<char>(SpeedRatio * vel_in);
//     }

//     Forwarder::Forwarder() : Node("vel_forwarder")
//     {
//         sub_cmd_vel = create_subscription<geometry_msgs::msg::Twist>(
//             "/cmd_vel", 10, 
//             std::bind(&Forwarder::cmdvelCallback, this, std::placeholders::_1));
        
//         pub_ly_vel = create_publisher<gimbal_driver::msg::Vel>(
//             "/ly/navi/vel", 10);
//     }

//     void Forwarder::cmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
//     {
//         RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear.x=%.2f, linear.y=%.2f", 
//                     msg->linear.x, msg->linear.y);
//         auto ly_msg = gimbal_driver::msg::Vel();
        
//         // 注意：坐标系转换，xy 交换，x 取反
//         // ly_msg.x = -speed_mapping(msg->linear.y);
//         // ly_msg.y = speed_mapping(msg->linear.x);
//         ly_msg.x = speed_mapping(msg->linear.x)*3;
//         ly_msg.y = speed_mapping(msg->linear.y)*3;


//         RCLCPP_INFO(this->get_logger(), "Mapped ly_vel: x=%d, y=%d", 
//                     static_cast<int>(ly_msg.x), static_cast<int>(ly_msg.y));
//         RCLCPP_DEBUG(this->get_logger(), "Forwarder: X: %d, Y: %d", 
//                      static_cast<int>(ly_msg.x), static_cast<int>(ly_msg.y));
        
        
//         pub_ly_vel->publish(ly_msg);
//     }
// } // namespace vel_forwarder

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<vel_forwarder::Forwarder>());
//     rclcpp::shutdown();
//     return 0;
// }
#include "forwarder.h"

namespace vel_forwarder
{
    constexpr float MaxSpeed = 2.0f;
    constexpr char MaxSpeedLevel = 100;
    constexpr float SpeedRatio = MaxSpeedLevel / MaxSpeed;
    constexpr float MinSpeed = 0.3f;

    char speed_mapping(float vel_in)
    {
        if (vel_in > 10.0f)
        {
            RCLCPP_WARN(rclcpp::get_logger("forwarder"), "Bad speed input: %.2f", vel_in);
            return 0;
        }

        vel_in = vel_in > 2.4f ? 2.4f : (vel_in < -2.4f ? -2.4f : vel_in);
        if (vel_in > -0.05f && vel_in < 0.05f)
        {
            vel_in = 0.0;
        }
        // if (vel_in > 0.0f && vel_in < MinSpeed)
        // {
        //     vel_in = MinSpeed;
        // }
        // else if (vel_in < 0.0f && vel_in > -MinSpeed)
        // {
        //     vel_in = -MinSpeed;
        // }
        
        return static_cast<char>(SpeedRatio * vel_in);
    }

    Forwarder::Forwarder() : Node("vel_forwarder")
    {
        sub_cmd_vel = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 1,
            std::bind(&Forwarder::cmdvelCallback, this, std::placeholders::_1));

        pub_ly_vel = create_publisher<gimbal_driver::msg::Vel>(
            "/ly/navi/vel", 10);
    }

    void Forwarder::cmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear.x=%.2f, linear.y=%.2f",
                    msg->linear.x, msg->linear.y);
        auto ly_msg = gimbal_driver::msg::Vel();

        // 注意：坐标系转换，xy 交换，x 取反
        // ly_msg.x = -speed_mapping(msg->linear.y);
        // ly_msg.y = speed_mapping(msg->linear.x);
        ly_msg.x = speed_mapping(msg->linear.x) ;
        ly_msg.y = speed_mapping(msg->linear.y) ;

        RCLCPP_INFO(this->get_logger(), "Mapped ly_vel: x=%d, y=%d",
                    static_cast<int>(ly_msg.x), static_cast<int>(ly_msg.y));
        RCLCPP_DEBUG(this->get_logger(), "Forwarder: X: %d, Y: %d",
                     static_cast<int>(ly_msg.x), static_cast<int>(ly_msg.y));

        pub_ly_vel->publish(ly_msg);
    }
} // namespace vel_forwarder

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vel_forwarder::Forwarder>());
    rclcpp::shutdown();
    return 0;
}