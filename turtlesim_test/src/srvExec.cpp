#include "rclcpp/rclcpp.hpp"
#include "angle_service.cpp"

int main() {
  rclcpp::init(0, nullptr);
  auto angle_server = std::make_shared<AngleService>();
  rclcpp::spin(angle_server);
  rclcpp::shutdown();
  return 0;
}