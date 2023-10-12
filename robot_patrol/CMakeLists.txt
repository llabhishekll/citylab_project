cmake_minimum_required(VERSION 3.5)
project(robot_patrol)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_patrol_interface REQUIRED)

# build
add_executable(robot_patrol_node src/patrol.cpp)
ament_target_dependencies(robot_patrol_node rclcpp sensor_msgs geometry_msgs)

add_executable(direction_service_node src/direction_service.cpp)
ament_target_dependencies(direction_service_node rclcpp robot_patrol_interface)

add_executable(test_service_node src/test_service.cpp)
ament_target_dependencies(test_service_node rclcpp robot_patrol_interface)

add_executable(patrol_with_service_node src/patrol_with_service.cpp)
ament_target_dependencies(patrol_with_service_node rclcpp robot_patrol_interface sensor_msgs geometry_msgs)

add_executable(go_to_pose_action_node src/go_to_pose_action.cpp)
ament_target_dependencies(go_to_pose_action_node rclcpp rclcpp_action robot_patrol_interface nav_msgs geometry_msgs)

# install
install(TARGETS robot_patrol_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS direction_service_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS test_service_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS patrol_with_service_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS go_to_pose_action_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
