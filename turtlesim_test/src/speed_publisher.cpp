#include <chrono>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

class SpeedPublisher : public rclcpp::Node {
public:
    SpeedPublisher() : Node("speed_publisher") {
        this->declare_parameter("linear_velocity", 0.0);
        this->declare_parameter("angular_velocity", 0.0);
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&SpeedPublisher::timer_callback, this));

        this->add_on_set_parameters_callback(std::bind(&SpeedPublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        auto message = geometry_msgs::msg::Twist();
        auto linear_velocity = this->get_parameter("linear_velocity").as_double();
        auto angular_velocity = this->get_parameter("angular_velocity").as_double();
        
        message.linear.x = linear_velocity;
        message.angular.z = angular_velocity;
        RCLCPP_INFO(this->get_logger(), "Publishing: Linear Velocity = %f", message.linear.x);
        RCLCPP_INFO(this->get_logger(), "Publishing: Angular Velocity = %f", message.angular.z);
        publisher_->publish(std::move(message));
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<SpeedPublisher>());
//     rclcpp::shutdown();
//     return 0;
// }