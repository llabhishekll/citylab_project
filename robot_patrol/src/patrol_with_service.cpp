#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_patrol_interface/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class PatrolWithService : public rclcpp::Node {
private:
  // member variables
  sensor_msgs::msg::LaserScan::SharedPtr last_laser_;

  // callback groups
  rclcpp::CallbackGroup::SharedPtr callback_g1;
  rclcpp::CallbackGroup::SharedPtr callback_g2;

  // ros objects
  rclcpp::Client<robot_patrol_interface::srv::GetDirection>::SharedPtr service_client;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_scan;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel;
  rclcpp::TimerBase::SharedPtr timer_robot_control;

  // member method
  void subscriber_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    this->last_laser_ = msg;
  }

  void timer_robot_control_callback() {
    int counter = 0;
    while (!this->service_client->wait_for_service(std::chrono::seconds(1))) {
      {
        // critical for closing infinite loop
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Terminating: interrupt received");
          return;
        }
        counter++;
        RCLCPP_WARN(this->get_logger(),
                    "Waiting for server /direction_service %d", counter);

        // halt robot in case of service non availability
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0;
        message.angular.z = 0;
        publisher_cmd_vel->publish(message);
      }
    }

    // define request body
    auto request =
        std::make_shared<robot_patrol_interface::srv::GetDirection::Request>();
    request->laser_data = *this->last_laser_;

    // send request to service server
    auto future = service_client->async_send_request(
        request, std::bind(&PatrolWithService::server_response_callback, this,
                           std::placeholders::_1));
  }

  void server_response_callback(
      rclcpp::Client<robot_patrol_interface::srv::GetDirection>::SharedFuture
          future) {
    // check for service server response availability
    auto status = future.wait_for(std::chrono::seconds(1));
    if (status == std::future_status::ready) {
      // get response body
      auto response = future.get();
      auto direction = response->direction.c_str();

      // publishing velocity to the topic /cmd_vel
      auto message = geometry_msgs::msg::Twist();

      // control structure
      if (strcmp(direction, "left") == 0) {
        message.linear.x = 0.1;
        message.angular.z = 0.5;
      } else if (strcmp(direction, "right") == 0) {
        message.linear.x = 0.1;
        message.angular.z = -0.5;
      } else if (strcmp(direction, "forward") == 0) {
        message.linear.x = 0.1;
        message.angular.z = 0;
      } else {
        message.linear.x = 0;
        message.angular.z = 0;
        RCLCPP_ERROR(this->get_logger(), "Error: unknown response");
      }

      // publisher feedback
      RCLCPP_INFO(this->get_logger(), "Direction : %s (L%f, A%f)", direction,
                  message.linear.x, message.angular.z);

      // publish velocity
      publisher_cmd_vel->publish(message);
    }
  }

public:
  // constructor
  PatrolWithService() : Node("patrol_with_service_node") {
    // callback groups objects
    callback_g1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_g2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // ros objects
    rclcpp::SubscriptionOptions sub_callback_g1;
    sub_callback_g1.callback_group = callback_g1;
    this->subscriber_scan =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&PatrolWithService::subscriber_callback, this,
                      std::placeholders::_1),
            sub_callback_g1);
    this->service_client =
        this->create_client<robot_patrol_interface::srv::GetDirection>(
            "/direction_service");
    this->publisher_cmd_vel =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    this->timer_robot_control = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PatrolWithService::timer_robot_control_callback, this),
        callback_g2);

    // node acknowledgement
    RCLCPP_INFO(this->get_logger(),
                "The patrol_with_service_node started successfully");
  }
};

int main(int argc, char *argv[]) {
  // initialize ros, executor and node
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<PatrolWithService> node =
      std::make_shared<PatrolWithService>();

  // add node to executor and spin
  executor.add_node(node);
  executor.spin();

  // shutdown
  rclcpp::shutdown();
  return 0;
}