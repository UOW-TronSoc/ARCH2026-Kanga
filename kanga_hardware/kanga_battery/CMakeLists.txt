cmake_minimum_required(VERSION 3.10)
project(kanga_battery)

# Warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(kanga_canbus REQUIRED)  # library-only package

# ---- Messages (local to this package) ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryInfo.msg"
  "msg/BmsStatus.msg"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# If you need the C++ typesupport target name for linking:
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# ---- Node ----
add_executable(daly_can_node
  src/daly_can_node.cpp
  src/main.cpp
)

target_compile_features(daly_can_node PRIVATE cxx_std_20)

# Your own public headers
target_include_directories(daly_can_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Pull in include dirs & link flags via ament for kanga_canbus and rclcpp
ament_target_dependencies(daly_can_node
  rclcpp
  kanga_canbus
)

# Link your generated message typesupport
target_link_libraries(daly_can_node
  ${cpp_typesupport_target}
)

# Ensure messages build before the node
add_dependencies(daly_can_node
  ${PROJECT_NAME}__cpp
)

# ---- Install ----
install(TARGETS daly_can_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
