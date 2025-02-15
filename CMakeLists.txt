cmake_minimum_required(VERSION 3.5)
project(livox_pointcloud_converter)
cmake_policy(SET CMP0074 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)

add_definitions(${PCL_DEFINITIONS})

add_compile_options(-march=native -O3 -fopenmp)
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(converter_node src/converter_node.cpp)

# Explicitly link OpenMP
target_link_libraries(converter_node OpenMP::OpenMP_CXX)
ament_target_dependencies(converter_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  PCL
)


install(TARGETS
  converter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
