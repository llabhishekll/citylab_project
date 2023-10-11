#include "rclcpp/rclcpp.hpp"
#include "robot_patrol_interface/srv/get_direction.hpp"

class DirectionService : public rclcpp::Node {
private:
  // ros objects
  rclcpp::Service<robot_patrol_interface::srv::GetDirection>::SharedPtr
      service_server;

  // member method
  void service_callback(
      const std::shared_ptr<robot_patrol_interface::srv::GetDirection::Request>
          request,
      const std::shared_ptr<robot_patrol_interface::srv::GetDirection::Response>
          response) {

    // publisher feedback
    RCLCPP_INFO(this->get_logger(),
                "/direction_service has received a new request.");

    // select accurate range
    auto f_pos = request->laser_data.ranges.begin() + 180;
    auto l_pos = request->laser_data.ranges.begin() + 540 + 1;

    // slice required vector
    std::vector<float> range_prime(l_pos - f_pos + 1);
    std::copy(f_pos, l_pos, range_prime.begin());

    // fill infinity with zero
    std::replace(range_prime.begin(), range_prime.end(),
                 std::numeric_limits<double>::infinity(), 0.0);

    // select direction range
    auto left_pos = range_prime.begin() + 360;
    auto frontl_pos = range_prime.begin() + 240;
    auto frontr_pos = range_prime.begin() + 120;
    auto right_pos = range_prime.begin() + 0;

    // not using reduce coz c++14
    auto total_dist_sec_left = std::accumulate(frontl_pos, left_pos, 0.0);
    auto total_dist_sec_front = std::accumulate(frontr_pos, frontl_pos, 0.0);
    auto total_dist_sec_right = std::accumulate(right_pos, frontr_pos, 0.0);

    // return service call
    if ((total_dist_sec_left > total_dist_sec_front) &&
        (total_dist_sec_left > total_dist_sec_right)) {
      response->direction = "left";
    } else if (total_dist_sec_front > total_dist_sec_right) {
      response->direction = "forward";
    } else {
      response->direction = "right";
    }
    RCLCPP_INFO(this->get_logger(), "Total Distance (L%f, M%f, R%f)",
                total_dist_sec_left, total_dist_sec_front,
                total_dist_sec_right);
  }

public:
  // constructor
  DirectionService() : Node("direction_service_node") {
    // ros objects
    this->service_server =
        create_service<robot_patrol_interface::srv::GetDirection>(
            "/direction_service",
            std::bind(&DirectionService::service_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

    // node acknowledgement
    RCLCPP_INFO(this->get_logger(),
                "The service /direction_service is available for request.");
  }
};

int main(int argc, char *argv[]) {
  // initialize ros
  rclcpp::init(argc, argv);

  // spin node
  rclcpp::spin(std::make_shared<DirectionService>());

  // shutdown
  rclcpp::shutdown();
  return 0;
}