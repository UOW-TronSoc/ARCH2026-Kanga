cmake_minimum_required(VERSION 3.10)
project(kanga_core)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(suspension_node src/suspension_node.cpp)
target_compile_features(suspension_node PRIVATE cxx_std_17)
ament_target_dependencies(suspension_node
  rclcpp
  sensor_msgs
)

install(TARGETS suspension_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
