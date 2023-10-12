#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_patrol_interface/action/go_to_pose.hpp"

class GoToPose : public rclcpp::Node {
private:
  // substitution
  using GoToPoseInterface = robot_patrol_interface::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseInterface>;

  // member variables
  geometry_msgs::msg::Pose2D current_pos_;
  geometry_msgs::msg::Pose2D desired_pos_;

  // ros objects
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel;
  rclcpp_action::Server<GoToPoseInterface>::SharedPtr action_server;

  // member method
  void subscriber_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    // reading current position from /odom topic
    this->current_pos_.x = msg->pose.pose.position.x;
    this->current_pos_.y = msg->pose.pose.position.y;

    // convert quaternion into euler angles
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;

    double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    this->current_pos_.theta = (180 / M_PI) * yaw;
  }

  rclcpp_action::GoalResponse action_handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const GoToPoseInterface::Goal> goal_value) {
    RCLCPP_INFO(this->get_logger(), "Request: Target Position :(X%f, Y%f, Z%f)",
                goal_value->goal_pos.x, goal_value->goal_pos.y,
                goal_value->goal_pos.theta);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  action_handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Terminating: interrupt received");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void action_handle_accepted(
      const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    std::thread{std::bind(&GoToPose::action_handle_execute, this,
                          std::placeholders::_1),
                goal_handle}
        .detach();
  }

  void
  action_handle_execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    // action acknowledgement
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    // define variables
    bool goal_done = false;
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<GoToPoseInterface::Result>();
    auto feedback = std::make_shared<GoToPoseInterface::Feedback>();
    auto message = geometry_msgs::msg::Twist();

    // control loop
    rclcpp::Rate loop_rate(1);
    while (!goal_done && rclcpp::ok()) {
      // if goal canceled
      if (goal_handle->is_canceling()) {
        result->status = true;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // goal logic
      goal_done = true;
    }

    // on completion halt robot and return result
    if (rclcpp::ok()) {
      result->status = true;
      message.linear.x = 0;
      message.angular.z = 0;
      publisher_cmd_vel->publish(message);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

public:
  // constructor
  explicit GoToPose(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("go_to_pose_action_node", node_options) {

    // ros objects
    this->subscriber_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&GoToPose::subscriber_callback, this, std::placeholders::_1));
    this->publisher_cmd_vel =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);        
    this->action_server = rclcpp_action::create_server<GoToPoseInterface>(
        this, "/go_to_pose",
        std::bind(&GoToPose::action_handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&GoToPose::action_handle_cancel, this, std::placeholders::_1),
        std::bind(&GoToPose::action_handle_accepted, this,
                  std::placeholders::_1));

    // node acknowledgement
    RCLCPP_INFO(this->get_logger(),
                "The go_to_pose_action_node started successfully");
  }
};

int main(int argc, char *argv[]) {
  // initialize ros, executor and node
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<GoToPose> node = std::make_shared<GoToPose>();

  // add node to executor and spin
  executor.add_node(node);
  executor.spin();

  // shutdown
  rclcpp::shutdown();
  return 0;
}