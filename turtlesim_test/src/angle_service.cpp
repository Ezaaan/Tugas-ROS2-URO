#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim_interface/srv/angle.hpp"

class AngleService : public rclcpp::Node {
public:
  AngleService() : Node("theta_service") {
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
        "turtle1/pose", 10, std::bind(&AngleService::pose_callback, this, std::placeholders::_1));
    service_ = this->create_service<turtlesim_interface::srv::Angle>(
        "absolute_angle", std::bind(&AngleService::absoluteAngle, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  float x, y;

  void pose_callback(const turtlesim::msg::Pose::SharedPtr message) {
    x = message->x;
    y = message->y;
  }

  void absoluteAngle(const std::shared_ptr <turtlesim_interface::srv::Angle::Request> request,
                std::shared_ptr <turtlesim_interface::srv::Angle::Response> response) {
    float goal_x = request->x;
    float goal_y = request->y;

    RCLCPP_INFO(this->get_logger(), "Received request with x: %f, y: %f", goal_x, goal_y);
    RCLCPP_INFO(this->get_logger(), "Current position is x: %f, y: %f", this->x, this->y);


    response->angle_resp = atan((goal_y - this->y) / (goal_x - this->x));

    // if (goal_x - this->x < 0 && goal_y - this->y < 0) {
    //   response->angle_resp += 3.14159;
    // }

    // if (response->angle_resp < 0) {
    //   response->angle_resp += 6.28319;
    // }
    // if (response->angle_resp > 6.28319) {
    //   response->angle_resp -= 6.28319;
    // }

    RCLCPP_INFO(this->get_logger(), "Sending response with theta: %f", response->angle_resp);
  }

  rclcpp::Service<turtlesim_interface::srv::Angle>::SharedPtr service_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
};

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<AngleService>());
//     rclcpp::shutdown();
//     return 0;
// }