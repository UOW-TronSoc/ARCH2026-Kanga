cmake_minimum_required(VERSION 3.10)
project(canbus_utils)

add_library(${PROJECT_NAME}
  src/socket_can.cpp
  src/epoll_event_loop.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
