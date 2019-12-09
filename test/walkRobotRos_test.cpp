/**
 /* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file walkRobotRos_test.cpp
 * @brief Source code for the implementation of walkRobotRos tests
 * @author Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 */

#include "walkRobotRos.hpp"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include <unistd.h>

/// Struct to count the callback function was called
struct TestSubHelper {
  TestSubHelper(): count(0) {}
  void Callback(const geometry_msgs::Twist msg) {
  count++;
}
  int count;
};

/**
 * @brief Test removeModels() of modelHandlingRos class
 *
 * @param[in]     walkRobotRosTest
 * @param[in]     TestLaserSensorCallback
 *
 * @return     none
 */

TEST(walkRobotRosTest, TestLaserSensorCallback) {
/// nodehandle ROS
  ros::NodeHandle nh;
/// Generating publisher to topic /scan
  ros::Publisher testWalkPublisher = nh.advertise<sensor_msgs::LaserScan>("/scan", 50);
/// Generating a laser mensage to be published at topic /scan
  sensor_msgs::LaserScan msgScan;
  msgScan.angle_min = -1.74;
  msgScan.angle_max = 1.74;
  msgScan.angle_increment = 0.0012;
  msgScan.time_increment = 0.0;
  msgScan.range_min = 0.1;
  msgScan.range_max = 5.0;
  msgScan.ranges.resize(5);
  msgScan.intensities.resize(5);

/// Setting the values og ranges to 0.1 to be detected as obstacle
  msgScan.ranges = {0.1, 0.1, 0.1, 0.1, 0.1};

/// Create object of class walkRobotRos
  walkRobotRos robot;

  int count = 0;

/// loop Rate
  ros::Rate loop_rate(5);

  while (ros::ok()) {
/// publish test
    testWalkPublisher.publish(msgScan);

/// Break loop after 11 iterations to generate new random value
    if (count == 11) {
      break;
    }
    ros::spinOnce();
    count++;
    loop_rate.sleep();
    }

/// Test if obstacle was detected
  EXPECT_TRUE(robot.walk.obstacle);
}

/**
 * @brief Test removeModels() of modelHandlingRos class
 *
 * @param[in]     walkRobotRosTest
 * @param[in]     TestWalkRobotRos
 *
 * @return     none
 */

TEST(walkRobotRosTest, TestWalkRobotRos) {
/// Initilize instance of a struct
  TestSubHelper helper;

/// Create object of class walkRobotRos
  walkRobotRos robotTwo;

/// nodehandle ROS
  ros::NodeHandle n;
/// Generating a velocity subscriber
  ros::Subscriber testVelSubscriber = n.subscribe<geometry_msgs::Twist>("/mobile_base/commands/velocity", 50, &TestSubHelper::Callback, &helper);

  ros::Rate loop_rate(5);

  int counter = 0;

  while (ros::ok()) {
/// publish test
    loop_rate.sleep();
    if (counter == 3) {
      break;
    }
    counter++;
    robotTwo.robotWalkRos();
    ros::spinOnce();
  }
  EXPECT_TRUE(1 < helper.count);
}
