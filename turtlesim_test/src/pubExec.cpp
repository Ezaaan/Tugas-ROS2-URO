#include "rclcpp/rclcpp.hpp"
#include "speed_publisher.cpp"

int main() {
  rclcpp::init(0, nullptr);
  auto speed_publisher = std::make_shared<SpeedPublisher>();
  rclcpp::spin(speed_publisher);
  rclcpp::shutdown();
  return 0;
}