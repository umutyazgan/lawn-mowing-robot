cmake_minimum_required(VERSION 2.8.3)
project(lawn_mowing_robot)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gmapping
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  gazebo_ros
  eigen_conversions
)

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_map_publisher
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs nav_msgs gazebo_ros eigen_conversions
  DEPENDS gazebo
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(lawn_map_publisher src/lawn_map_publisher.cpp)
# add_executable(lawn_explorer src/lawn_explorer.py)
# add_executable(gazebo_map_publisher src/GazeboMapPublisher.cpp)
add_library(gazebo_map_publisher
   src/GazeboMapPublisher.cpp
)

add_dependencies(gazebo_map_publisher ${catkin_EXPORTED_TARGETS})

target_link_libraries(lawn_map_publisher ${catkin_LIBRARIES})
# target_link_libraries(lawn_explorer ${catkin_LIBRARIES})

