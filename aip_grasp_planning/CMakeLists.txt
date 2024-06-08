cmake_minimum_required(VERSION 3.8)
project(aip_grasp_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(aip_grasp_planning_interfaces REQUIRED)


#find directory to includ .h files
include_directories(
  include 
  ${pcl_processing_INCLUDE_DIRS})

 
set(DEPENDENCIES 
  rclcpp
  ament_index_cpp
  pcl_ros
  sensor_msgs
  aip_grasp_planning_interfaces
)

file (GLOB SOURCES_NODES src/aip_grasp_planning/*.cpp)
file (GLOB SOURCES_ACTIONS src/actions/*.cpp)
file (GLOB SOURCES_SERVICES src/services/*.cpp)
file (GLOB SOURCES_CONDITIONS src/conditions/*.cpp)
file (GLOB SOURCES_CONTROLS src/controls/*.cpp)
file (GLOB SOURCES_NODES src/nodes/*.cpp)

add_executable(point_cloud_processing_node
  src/point_cloud_processing_node.cpp
  ${SOURCES_ACTIONS}
  ${SOURCES_SERVICES}
  ${SOURCES_CONDITIONS}
  ${SOURCES_CONTROLS}
  ${SOURCES_NODES})

ament_target_dependencies(point_cloud_processing_node
  ${DEPENDENCIES})

ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/grasp_planning_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/)


install(TARGETS 
  point_cloud_processing_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()