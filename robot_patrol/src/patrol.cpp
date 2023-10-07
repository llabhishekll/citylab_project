#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class Patrol : public rclcpp::Node {
private:
  // member variables
  int direction_;
  float l_dist;
  float c_dist;
  float r_dist;
  float l_view_dist;
  float r_view_dist;

  // ros objects
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_scan;

  // member method
  void subscriber_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // select accurate range
    auto f_pos = msg->ranges.begin() + 180;
    auto l_pos = msg->ranges.begin() + 540 + 1;

    // slice required vector
    std::vector<float> range_prime(l_pos - f_pos + 1);
    std::copy(f_pos, l_pos, range_prime.begin());

    // calculate distance
    this->l_dist = range_prime[360];
    this->l_view_dist = range_prime[198]; // 81
    this->c_dist = range_prime[180];      // 90
    this->r_view_dist = range_prime[162]; // 99
    this->r_dist = range_prime[0];

    // fill infinity with zero
    std::replace(range_prime.begin(), range_prime.end(),
                 std::numeric_limits<double>::infinity(), 0.0);

    // find max position
    auto max_value = std::max_element(range_prime.begin(), range_prime.end());
    auto max_pos = std::distance(range_prime.begin(), max_value);

    // calculate angle
    this->direction_ = (max_pos - 180) / 2;

    // subscriber feedback
    RCLCPP_INFO(this->get_logger(), "LaserScan (L%f, M%f, R%f)", this->l_dist,
                this->c_dist, this->r_dist);
    RCLCPP_INFO(this->get_logger(), "MaxElement (%d, %f) Angle (%d)", max_pos,
                *max_value, this->direction_);
  }

public:
  // constructor
  Patrol() : Node("robot_patrol_node") {
    // ros objects
    subscriber_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
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