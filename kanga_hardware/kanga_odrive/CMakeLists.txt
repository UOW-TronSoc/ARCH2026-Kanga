cmake_minimum_required(VERSION 3.10)
project(kanga_odrive)

# Warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Deps
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(kanga_canbus REQUIRED)      # library-only package you created
find_package(kanga_interfaces REQUIRED)  # your shared interfaces (if used)

# Executable
add_executable(odrive_can_node
  src/odrive_can_node.cpp
  src/main.cpp
)

target_compile_features(odrive_can_node PRIVATE cxx_std_20)

# Your own public headers
target_include_directories(odrive_can_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Bring in include dirs and link flags from deps
ament_target_dependencies(odrive_can_node
  rclcpp
  std_srvs
  kanga_canbus
  kanga_interfaces
)

# Install only your targets
install(TARGETS odrive_can_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
