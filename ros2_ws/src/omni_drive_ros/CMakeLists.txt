cmake_minimum_required(VERSION 3.16)
project(omni_drive_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
foreach(dep rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs
            diagnostic_msgs tf2 tf2_ros tf2_geometry_msgs)
  find_package(${dep} REQUIRED)
endforeach()

# Build the control core from the repository root. It has no ROS dependency
# whatsoever -- that is the whole point of the layering -- so it is pulled in
# as a plain CMake subdirectory rather than an ament package.
set(ODC_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(../../.. odc_build)

add_executable(omni_drive_node src/omni_drive_node.cpp)
target_link_libraries(omni_drive_node odc)
ament_target_dependencies(omni_drive_node
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs
  diagnostic_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS omni_drive_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
