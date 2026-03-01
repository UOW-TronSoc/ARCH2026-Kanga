cmake_minimum_required(VERSION 3.8)
project(kanga_arm_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

# Standard ROS2 stuffs
find_package(ament_cmake REQUIRED)

option(BUILD_RAISIM_BRIDGE "Build the RaiSim bridge node" ON)

if(BUILD_RAISIM_BRIDGE)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  # RaiSim stuffs
  set(RAISIM_DEFAULT_PREFIX "/home/hunter/Documents/github/importedRepos/raisimLib/raisim/linux")
  if(EXISTS "${RAISIM_DEFAULT_PREFIX}")
    list(PREPEND CMAKE_PREFIX_PATH "${RAISIM_DEFAULT_PREFIX}")
  endif()

  find_package(raisim QUIET)
  if(raisim_FOUND)
    find_package(Eigen3 REQUIRED)
    find_package(PythonLibs 3.10 REQUIRED)  # Change Python version as needed
    include_directories(
        ${EIGEN3_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        # "/home/hunter/Documents/github/importedRepos/matplotlib-cpp"
    )

    add_executable(raisim_bridge src/raisim_bridge.cpp)
    ament_target_dependencies(raisim_bridge rclcpp sensor_msgs std_msgs Eigen3 raisim PythonLibs)
  else()
    message(WARNING "raisim not found; skipping raisim_bridge. Set raisim_DIR or CMAKE_PREFIX_PATH to enable.")
  endif()
endif()

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

# Install
if(TARGET raisim_bridge)
  install(TARGETS
    raisim_bridge
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
