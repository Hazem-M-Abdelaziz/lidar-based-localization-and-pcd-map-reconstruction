cmake_minimum_required(VERSION 3.8)
project(lidar_pose_estimator)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-O3 -Wall -Wextra)

cmake_policy(SET CMP0074 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)


set(COMMON_INCLUDE_DIRS
  include
  ${PCL_INCLUDE_DIRS})

set(COMMON_DEPS
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_srvs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions)


#LiDAR pose estimator

add_executable(lidar_pose_estimator_node
  src/lidar_pose_estimator_node.cpp)

target_include_directories(lidar_pose_estimator_node PRIVATE
  ${COMMON_INCLUDE_DIRS})

target_compile_definitions(lidar_pose_estimator_node PRIVATE
  ${PCL_DEFINITIONS})

ament_target_dependencies(lidar_pose_estimator_node
  ${COMMON_DEPS})

target_link_libraries(lidar_pose_estimator_node
  ${PCL_LIBRARIES})


#Evaluation node

add_executable(evaluation_node
  src/evaluation_node.cpp)

target_include_directories(evaluation_node PRIVATE
  ${COMMON_INCLUDE_DIRS})

target_compile_definitions(evaluation_node PRIVATE
  ${PCL_DEFINITIONS})

ament_target_dependencies(evaluation_node
  ${COMMON_DEPS})

target_link_libraries(evaluation_node
  ${PCL_LIBRARIES})


install(TARGETS
  lidar_pose_estimator_node
  evaluation_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
