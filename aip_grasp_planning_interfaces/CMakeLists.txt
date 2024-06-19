cmake_minimum_required(VERSION 3.5)
project(aip_grasp_planning_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(aip_packing_planning_interfaces)

set(DEPENDENCIES 
    sensor_msgs
    aip_packing_planning_interfaces
)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GraspObjectSurfaceNormal.srv"
  "srv/GraspPlanning.srv"
  DEPENDENCIES ${DEPENDENCIES}
)

ament_export_dependencies(rosidl_default_runtime ${DEPENDENCIES})
ament_package()
