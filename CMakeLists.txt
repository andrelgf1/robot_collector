cmake_minimum_required(VERSION 2.8.3)
project(robot_collector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
  std_msgs
  message_generation
)

find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtle_walker
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS gazebo_ros geometry_msgs roscpp std_msgs
  DEPENDS gazebo_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)


add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES})
