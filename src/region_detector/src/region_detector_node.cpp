#include "region_detector.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RegionDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
