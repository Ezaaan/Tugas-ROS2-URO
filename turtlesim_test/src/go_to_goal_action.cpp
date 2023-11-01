#include <functional>
#include <memory>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "turtlesim_interface/action/gtg.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim_interface/srv/angle.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/action/rotate_absolute.hpp"
#include "pose_subscriber.cpp"
#include "speed_publisher.cpp"

namespace action_gtg_cpp
{
class GTGActionServer : public rclcpp::Node
{
public:
  explicit GTGActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("gtg_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<turtlesim_interface::action::Gtg>(
      this,
      "GoToGoal",
      std::bind(&GTGActionServer::handle_goal, this, _1, _2),
      std::bind(&GTGActionServer::handle_cancel, this, _1),
      std::bind(&GTGActionServer::handle_accepted, this, _1));

    angle_client_ = this->create_client<turtlesim_interface::srv::Angle>("absolute_angle");
    rotate_client_ = rclcpp_action::create_client<turtlesim::action::RotateAbsolute>(this, "/turtle1/rotate_absolute");
  }

private:
  rclcpp_action::Server<turtlesim_interface::action::Gtg>::SharedPtr action_server_;
  rclcpp::Client<turtlesim_interface::srv::Angle>::SharedPtr angle_client_;
  rclcpp_action::Client<turtlesim::action::RotateAbsolute>::SharedPtr rotate_client_;
  std::shared_ptr <PoseSubscriber> pose_;
  std::shared_ptr <SpeedPublisher> speed_;

  rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const turtlesim_interface::action::Gtg::Goal> msg) {
      RCLCPP_INFO(this->get_logger(), "Goal has been recieved: %f, y: %f", msg->x, msg->y);
      (void) uuid;
      if (!validateGoal(msg->x, msg->y)) {
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<turtlesim_interface::action::Gtg>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
        const std::shared_ptr <rclcpp_action::ServerGoalHandle<turtlesim_interface::action::Gtg>> goal_handle) {
      std::thread{std::bind(&GTGActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<turtlesim_interface::action::Gtg>> goal_handle)
  {
    pose_ = std::make_shared<PoseSubscriber>();
    speed_ = std::make_shared<SpeedPublisher>();

    speed_ ->set_parameter(rclcpp::Parameter("linear_velocity", 0.5));
    speed_ ->set_parameter(rclcpp::Parameter("angular_velocity", 0.0));
    speed_->publish_speed();

    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    auto feedback = std::make_shared<turtlesim_interface::action::Gtg::Feedback>();
    auto result = std::make_shared<turtlesim_interface::action::Gtg::Result>();
    auto goal = goal_handle->get_goal();
    RCLCPP_INFO(this->get_logger(), "Stuck here");
    
    auto request = std::make_shared<turtlesim_interface::srv::Angle::Request>();
    request->x = goal->x;
    request->y = goal->y;
    auto angle_result = angle_client_->async_send_request(request).get();
    RCLCPP_INFO(this->get_logger(), "Stuck here 2");


    if (!angle_result) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get theta");
        result->is_completed = false;
        goal_handle->succeed(result);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Stuck here 3");


    // rotate to theta using service
    auto rotate = turtlesim::action::RotateAbsolute::Goal();
    rotate.theta = angle_result->angle_resp;
    RCLCPP_INFO(this->get_logger(), "Rotating to theta: %f", angle_result->angle_resp);
    auto rotate_result = rotate_client_->async_send_goal(rotate).get();

    if (!rotate_result) {
      RCLCPP_ERROR(this->get_logger(), "Failed to rotate");
      result->is_completed= false;
      goal_handle->succeed(result);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Rotated to theta: %f", angle_result->angle_resp);
    RCLCPP_INFO(this->get_logger(), "Moving to x: %f, y: %f", goal->x, goal->y);

    // move to goal
      while (!isGoalReached(pose_->getX(), pose_->getY(), goal->x, goal->y)) {
        RCLCPP_INFO(this->get_logger(), "Current x: %f, y: %f", pose_->getX(), pose_->getY());
        feedback->distance_to_goal = sqrt(pow(goal->x - pose_->getX(),2) + pow(goal->y - pose_->getY(),2));
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
      }

      // stop moving
      speed_->set_parameter(rclcpp::Parameter("linear_velocity", 0));
      speed_->set_parameter(rclcpp::Parameter("angular_velocity", 0));

      // set result
      result->is_completed= true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }

  bool validateGoal(float x, float y) {
      return x >= 0 && y >= 0 && x <= 10 && y <= 10;
  }

  bool isGoalReached(float curr_x, float curr_y, float goal_x, float goal_y) {
    return abs(curr_x - goal_x) < 0.2 && abs(curr_y - goal_y) < 0.2;
  }
};  // class GTGActionServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_gtg_cpp::GTGActionServer)