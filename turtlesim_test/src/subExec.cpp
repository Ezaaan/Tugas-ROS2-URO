#include "rclcpp/rclcpp.hpp"
#include "pose_subscriber.cpp"

int main() {
  rclcpp::init(0, nullptr);
  auto pose_subscriber = std::make_shared<PoseSubscriber>();
  rclcpp::spin(pose_subscriber);
  rclcpp::shutdown();
  return 0;
}