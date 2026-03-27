cmake_minimum_required(VERSION 3.8)
project(kanga_arm_joy_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(joy_to_joint_control src/joy_to_joint_control.cpp)
ament_target_dependencies(joy_to_joint_control rclcpp sensor_msgs)

add_executable(joy_to_world_state_control src/joy_to_world_state_control.cpp)
ament_target_dependencies(joy_to_world_state_control rclcpp sensor_msgs geometry_msgs)

add_executable(joy_to_dual_control src/joy_to_dual_control.cpp)
ament_target_dependencies(joy_to_dual_control rclcpp sensor_msgs geometry_msgs)

add_executable(joy_to_ee_state_control src/joy_to_ee_state_control.cpp)
ament_target_dependencies(joy_to_ee_state_control rclcpp sensor_msgs geometry_msgs)

add_executable(joy_to_hybrid_control src/joy_to_hybrid_control.cpp)
ament_target_dependencies(joy_to_hybrid_control rclcpp sensor_msgs geometry_msgs std_msgs)

install(TARGETS
  joy_to_joint_control
  joy_to_world_state_control
  joy_to_dual_control
  joy_to_ee_state_control
  joy_to_hybrid_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
