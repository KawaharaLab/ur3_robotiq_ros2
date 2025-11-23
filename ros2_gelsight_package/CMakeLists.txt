cmake_minimum_required(VERSION 3.5)
project(ros2_gelsight_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Install Python modules
install(PROGRAMS
  src/ros2_gelsight_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install configuration files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install additional resources
install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()