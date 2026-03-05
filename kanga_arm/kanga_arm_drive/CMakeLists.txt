cmake_minimum_required(VERSION 3.8)
project(kanga_arm_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kanga_interfaces REQUIRED)

add_executable(arm_command_mapper src/arm_command_mapper.cpp)
ament_target_dependencies(
  arm_command_mapper
  ament_index_cpp
  rclcpp
  sensor_msgs
  kanga_interfaces)

add_executable(arm_feedback_bridge src/arm_feedback_bridge.cpp)
ament_target_dependencies(
  arm_feedback_bridge
  ament_index_cpp
  rclcpp
  sensor_msgs
  kanga_interfaces)

add_executable(arm_end_effector_mapper src/arm_end_effector_mapper.cpp)
ament_target_dependencies(
  arm_end_effector_mapper
  can_msgs
  rclcpp
  sensor_msgs)

install(TARGETS
  arm_command_mapper
  arm_feedback_bridge
  arm_end_effector_mapper
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
