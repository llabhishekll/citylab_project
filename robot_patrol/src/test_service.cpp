#include "rclcpp/rclcpp.hpp"
#include "robot_patrol_interface/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class DirectionServiceClient : public rclcpp::Node {
private:
  // member variables
  sensor_msgs::msg::LaserScan::SharedPtr last_laser_;

  // callback groups
  rclcpp::CallbackGroup::SharedPtr callback_g1;
  rclcpp::CallbackGroup::SharedPtr callback_g2;

  // ros objects
  rclcpp::Client<robot_patrol_interface::srv::GetDirection>::SharedPtr
      service_client;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_scan;
  rclcpp::TimerBase::SharedPtr timer_service_client;

  // member method
  void subscriber_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    this->last_laser_ = msg;
  }

  void timer_service_client_callback() {
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
      }
    }

    // define request body
    auto request =
        std::make_shared<robot_patrol_interface::srv::GetDirection::Request>();
    request->laser_data = *this->last_laser_;

    // send request to service server
    auto future = service_client->async_send_request(
        request, std::bind(&DirectionServiceClient::server_response_callback,
                           this, std::placeholders::_1));
  }

  void server_response_callback(
      rclcpp::Client<robot_patrol_interface::srv::GetDirection>::SharedFuture
          future) {
    // check for service server response availability
    auto status = future.wait_for(std::chrono::seconds(1));
    if (status == std::future_status::ready) {
      // get response body
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Result: success {direction : %s}",
                  response->direction.c_str());
    }
  }

public:
  DirectionServiceClient() : Node("test_service_node") {
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
            std::bind(&DirectionServiceClient::subscriber_callback, this,
                      std::placeholders::_1),
            sub_callback_g1);
    this->service_client =
        this->create_client<robot_patrol_interface::srv::GetDirection>(
            "/direction_service");
    this->timer_service_client = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&DirectionServiceClient::timer_service_client_callback, this),
        callback_g2);

    // node acknowledgement
    RCLCPP_INFO(this->get_logger(),
                "The test_service_node started successfully");
  }
};

int main(int argc, char *argv[]) {
  // initialize ros, executor and node
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<DirectionServiceClient> node =
      std::make_shared<DirectionServiceClient>();

  // add node to executor and spin
  executor.add_node(node);
  executor.spin();

  // shutdown
  rclcpp::shutdown();
  return 0;
}