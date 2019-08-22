################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(robotis_manipulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(rclcpp REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Build
################################################################################
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

set(dependencies_lib
  "EIGEN3"
  "cmake_modules"
  "rclcpp"
)

set(LIB_NAME "robotis_manipulator")

add_library(${LIB_NAME} SHARED
  src/robotis_manipulator/robotis_manipulator.cpp
  src/robotis_manipulator/robotis_manipulator_common.cpp
  src/robotis_manipulator/robotis_manipulator_log.cpp
  src/robotis_manipulator/robotis_manipulator_trajectory_generator.cpp
  src/robotis_manipulator/robotis_manipulator_manager.cpp
  src/robotis_manipulator/robotis_manipulator_math.cpp
)

ament_target_dependencies(${LIB_NAME} ${dependencies_lib})
target_link_libraries(${LIB_NAME} ${Eigen3_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS ${LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

################################################################################
# Test
################################################################################

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(cmake_modules)
ament_export_dependencies(Eigen3)
ament_export_libraries(${LIB_NAME})
ament_package()