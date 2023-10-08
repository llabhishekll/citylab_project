#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#define GAMMA 0.45
#define X_VEL 0.10

class Patrol : public rclcpp::Node {
private:
  // member variables
  int direction_;
  float l_dist;
  float c_dist;
  float r_dist;
  float min_value;

  // ros objects
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_scan;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel;
  rclcpp::TimerBase::SharedPtr timer_robot_control;

  // member method
  void subscriber_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // select accurate range
    auto f_pos = msg->ranges.begin() + 180;
    auto l_pos = msg->ranges.begin() + 540 + 1;

    // slice required vector
    std::vector<float> range_prime(l_pos - f_pos + 1);
    std::copy(f_pos, l_pos, range_prime.begin());

    // calculate distance
    this->l_dist = range_prime[360]; // 180
    this->c_dist = range_prime[180]; // 90
    this->r_dist = range_prime[0];   // 0

    // find min position
    auto min_f_pos = range_prime.begin() + 156; // 78
    auto min_l_pos = range_prime.begin() + 204; // 102
    auto min_value = std::min_element(min_f_pos, min_l_pos);
    this->min_value = *min_value;

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

  void timer_robot_control_callback() {
    // publishing velocity to the topic /cmd_vel
    auto message = geometry_msgs::msg::Twist();
    // control structure
    if ((this->min_value < GAMMA)) {
      message.linear.x = 0;
      message.angular.z = ((M_PI / 180) * this->direction_) / 2;
    } else {
      message.linear.x = X_VEL;
      message.angular.z = 0;
    }
    // publisher feedback
    RCLCPP_INFO(this->get_logger(), "Velocity (L%f, A%f)", message.linear.x,
                message.angular.z);
    // publish velocity
    publisher_cmd_vel->publish(message);
  }

public:
  // constructor
  Patrol() : Node("robot_patrol_node") {
    // ros objects
    this->subscriber_scan =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&Patrol::subscriber_callback, this,
                      std::placeholders::_1));
    this->publisher_cmd_vel =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    this->timer_robot_control = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Patrol::timer_robot_control_callback, this));
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