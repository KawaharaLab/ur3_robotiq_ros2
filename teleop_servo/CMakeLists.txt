cmake_minimum_required(VERSION 3.8)
project(teleop_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robotiq_2f_gripper_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(teleop_servo_demo SHARED
  src/joystick_servo_ur.cpp
)

ament_target_dependencies(teleop_servo_demo
  control_msgs
  geometry_msgs
  moveit_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  robotiq_2f_gripper_msgs
  sensor_msgs
  std_srvs
)

rclcpp_components_register_nodes(teleop_servo_demo "teleop_servo::JoyToServoPubUr")

target_compile_features(teleop_servo_demo PUBLIC c_std_99 cxx_std_17)

install(TARGETS teleop_servo_demo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY src
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
