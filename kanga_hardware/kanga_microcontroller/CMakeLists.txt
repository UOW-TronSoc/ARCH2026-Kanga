cmake_minimum_required(VERSION 3.10)
project(kanga_microcontroller)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kanga_canbus REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MicrocontrollerFrame.msg"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(microcontroller_can_node
  src/microcontroller_can_node.cpp
  src/main.cpp
)

target_compile_features(microcontroller_can_node PRIVATE cxx_std_20)

target_include_directories(microcontroller_can_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(microcontroller_can_node
  rclcpp
  kanga_canbus
)

target_link_libraries(microcontroller_can_node
  ${cpp_typesupport_target}
)

add_dependencies(microcontroller_can_node
  ${PROJECT_NAME}__cpp
)

install(TARGETS microcontroller_can_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
