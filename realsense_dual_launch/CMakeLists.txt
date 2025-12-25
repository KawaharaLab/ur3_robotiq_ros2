cmake_minimum_required(VERSION 3.8)
project(realsense_dual_launch)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Ensure package manifest is installed so ament index can find the package
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Register package with ament resource index
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

ament_package()
