cmake_minimum_required(VERSION 3.10)
project(kanga_canbus)

find_package(ament_cmake REQUIRED)

add_library(kanga_canbus SHARED
  src/epoll_event_loop.cpp
  src/socket_can.cpp
)

target_include_directories(kanga_canbus PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Optional: create a namespaced alias that consumers can link to
add_library(kanga_canbus::kanga_canbus ALIAS kanga_canbus)

install(TARGETS kanga_canbus
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(kanga_canbus)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
