#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class Patrol : public rclcpp::Node {
private:
  // ros object
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_scan;
  sensor_msgs::msg::LaserScan::SharedPtr scan_prime;

  // member method
  void subscriber_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    this->scan_prime = msg;
    RCLCPP_INFO(this->get_logger(), "LaserScan (%f, %f, %f)", msg->ranges[180], msg->ranges[360], msg->ranges[540]);
  }

public:
  // constructor
  Patrol() : Node("robot_patrol_node") {
    // ros object
    subscriber_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&Patrol::subscriber_callback, this, std::placeholders::_1));

    // node acknowledgement
    RCLCPP_INFO(this->get_logger(),
                "The robot_patrol_node started successfully");
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}