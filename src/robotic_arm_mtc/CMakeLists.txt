cmake_minimum_required(VERSION 3.16)
project(robotic_arm_mtc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(generate_parameter_library REQUIRED)

find_package(moveit_common REQUIRED)
moveit_package()

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)

find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

# ------------------------------------------------------------------------------
# Generate parameters from yaml
# ------------------------------------------------------------------------------

generate_parameter_library(
  pick_place_parameters
  src/pick_place_parameters.yaml
)

# ------------------------------------------------------------------------------
# Include directories
# ------------------------------------------------------------------------------

include_directories(
  include
)

# ------------------------------------------------------------------------------
# Pick Place Task Library
# ------------------------------------------------------------------------------

add_library(
  robotic_arm_pick_place_task
  SHARED
  src/pick_place_task.cpp
)

target_link_libraries(
  robotic_arm_pick_place_task
  ${moveit_task_constructor_core_TARGETS}
  pick_place_parameters
)

ament_target_dependencies(
  robotic_arm_pick_place_task
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  tf2_eigen
)

# ------------------------------------------------------------------------------
# Main executable
# ------------------------------------------------------------------------------

add_executable(
  pick_place_demo
  src/pick_place.cpp
)

target_link_libraries(
  pick_place_demo
  robotic_arm_pick_place_task
  pick_place_parameters
)

ament_target_dependencies(
  pick_place_demo
  rclcpp
)

# ------------------------------------------------------------------------------
# Install targets
# ------------------------------------------------------------------------------

install(
  TARGETS
    robotic_arm_pick_place_task
    pick_place_demo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Install headers
# ------------------------------------------------------------------------------

install(
  DIRECTORY include/
  DESTINATION include
)

# ------------------------------------------------------------------------------
# Install config and launch files
# ------------------------------------------------------------------------------

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Export package
# ------------------------------------------------------------------------------

ament_package()