cmake_minimum_required(VERSION 3.10)
project(kanga_battery)

# Warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kanga_canbus REQUIRED)  # library-only package
find_package(kanga_interfaces REQUIRED)

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
  kanga_interfaces
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
