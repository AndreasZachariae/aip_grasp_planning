cmake_minimum_required(VERSION 3.8)
project(pcl_processing)

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


#find directory to includ .h files
include_directories(
  include 
  ${pcl_processing_INCLUDE_DIRS})

 
set(DEPENDENCIES 
  rclcpp
  ament_index_cpp
  pcl_ros
  sensor_msgs
)

file (GLOB SOURCES_NODES src/pcl_processing/*.cpp)

add_executable(pcl_subscriber
  src/basic_pcl_subscriber.cpp
  ${SOURCES_NODES})

ament_target_dependencies(pcl_subscriber
  ${DEPENDENCIES})

ament_python_install_package(scripts)

# Install Python executables
install(PROGRAMS
  scripts/transform_depth_image.py
  DESTINATION lib/${PROJECT_NAME}
)

# # Install launch files.
# install(DIRECTORY
#   launch
#   config
#   DESTINATION share/${PROJECT_NAME}/)


install(TARGETS 
pcl_subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()