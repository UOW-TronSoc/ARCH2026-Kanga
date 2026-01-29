cmake_minimum_required(VERSION 3.10)
project(kanga_core)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kanga_canbus REQUIRED)
find_package(kanga_microcontroller REQUIRED)

add_executable(core_can_node
  src/core_can_node.cpp
)

target_compile_features(core_can_node PRIVATE cxx_std_20)

ament_target_dependencies(core_can_node
  rclcpp
  kanga_canbus
  kanga_microcontroller
)

target_link_libraries(core_can_node
  kanga_microcontroller::microcontroller_can_node_lib
)

install(TARGETS core_can_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
