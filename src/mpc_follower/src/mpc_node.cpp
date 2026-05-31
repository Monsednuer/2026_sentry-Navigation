#include "mpc_controller.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mpc_follower::MpcController>());
    rclcpp::shutdown();
    return 0;
}