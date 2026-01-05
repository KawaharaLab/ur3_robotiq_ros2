cmake_minimum_required(VERSION 3.16)
project(vla_adapter)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

install(PROGRAMS
  scripts/pi0_inference_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ftvla_inference_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
