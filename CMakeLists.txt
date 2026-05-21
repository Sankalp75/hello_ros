cmake_minimum_required(VERSION 3.10)
project(robotic_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_executable(arm_controller_node
  src/arm_controller_node.cpp
)

ament_target_dependencies(arm_controller_node
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
)

install(TARGETS arm_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY
  launch
  urdf
  config
  world
  scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()