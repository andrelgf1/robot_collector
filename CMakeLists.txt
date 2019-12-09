cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 11)
project(robot_collector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  ar_track_alvar
  move_base
  gazebo_ros
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
  ar_track_alvar
  move_base
  gazebo
  DEPENDS
  gazebo_ros
)

add_executable(
  robot_collector 
  src/main.cpp
  src/collector.cpp
  src/collector_ros.cpp
  src/detector.cpp
  src/detector_ros.cpp
  src/modelHandling.cpp
  src/modelHandlingRos.cpp
  src/randCoord.cpp
  src/walkRobot.cpp
  src/walkRobotRos.cpp
  )
target_link_libraries(robot_collector ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")
  add_rostest_gtest(
    robot_collector_test
    test/test.launch
    test/main.cpp
    test/collector_test.cpp
    test/detector_ros_test.cpp
    test/detector_test.cpp
    test/modelHandling_test.cpp
    test/modelHandlingRos_test.cpp
    test/randCoord_test.cpp
    test/walkRobot_test.cpp
    test/walkRobotRos_test.cpp
    src/collector.cpp
    src/collector_ros.cpp
    src/detector.cpp
    src/detector_ros.cpp
    src/modelHandling.cpp
    src/modelHandlingRos.cpp
    src/randCoord.cpp
    src/walkRobot.cpp
    src/walkRobotRos.cpp
  )
  target_link_libraries(robot_collector_test ${catkin_LIBRARIES})
  add_dependencies(robot_collector_test robot_collector ${catkin_EXPORTED_TARGETS})
endif()
