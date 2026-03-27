cmake_minimum_required(VERSION 3.8)
project(kanga_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kanga_interfaces REQUIRED)

add_executable(wheel_command_mapper src/wheel_command_mapper.cpp)
ament_target_dependencies(wheel_command_mapper rclcpp geometry_msgs std_msgs kanga_interfaces)

add_executable(wheel_command_mapper_new src/wheel_command_mapper_new.cpp)
ament_target_dependencies(wheel_command_mapper_new rclcpp geometry_msgs std_msgs kanga_interfaces)

add_executable(twist_to_percent src/twist_to_percent.cpp)
ament_target_dependencies(twist_to_percent rclcpp geometry_msgs)

install(TARGETS
  wheel_command_mapper
  wheel_command_mapper_new
  twist_to_percent
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})


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

ament_package()
