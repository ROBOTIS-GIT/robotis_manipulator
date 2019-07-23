################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(robotis_manipulator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
## Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare ament specific configuration to be passed to dependent projects
################################################################################
ament_export_dependencies(rclcpp)
ament_export_dependencies(cmake_modules)
ament_export_dependencies(Eigen3)

################################################################################
# Build
################################################################################
include_directories(
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(robotis_manipulator
  src/robotis_manipulator/robotis_manipulator.cpp
  src/robotis_manipulator/robotis_manipulator_common.cpp
  src/robotis_manipulator/robotis_manipulator_log.cpp
  src/robotis_manipulator/robotis_manipulator_trajectory_generator.cpp
  src/robotis_manipulator/robotis_manipulator_manager.cpp
  src/robotis_manipulator/robotis_manipulator_math.cpp
)

ament_target_dependencies(robotis_manipulator rclcpp cmake_modules EIGEN3)
target_link_libraries(robotis_manipulator ${rclcpp_LIBRARIES} ${cmake_modules_LIBRARIES} ${Eigen3_LIBRARIES})

################################################################################
# Install
################################################################################
install(
  TARGETS robotis_manipulator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(robotis_manipulator)
ament_package()