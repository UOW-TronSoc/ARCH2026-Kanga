cmake_minimum_required(VERSION 3.10)
project(kanga_cameras)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(camera_publisher_node src/camera_publisher_node.cpp)
target_compile_features(camera_publisher_node PRIVATE cxx_std_17)
ament_target_dependencies(camera_publisher_node
  rclcpp
  sensor_msgs
  ament_index_cpp
)
# cv_bridge brings OpenCV; we need opencv_videoio for VideoCapture
find_package(OpenCV REQUIRED)
target_include_directories(camera_publisher_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_publisher_node ${OpenCV_LIBS} yaml-cpp)

install(TARGETS camera_publisher_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
