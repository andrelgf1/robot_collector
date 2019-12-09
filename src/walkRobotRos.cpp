/**
 /* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file walkRobotRos.cpp
 * @brief Source code for the implementation of walkRobotRos class
 * @author Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 */

#include "walkRobotRos.hpp"

#include <stdlib.h>
#include <ros/ros.h>
#include <time.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"

#include "walkRobot.hpp"

walkRobotRos::walkRobotRos() {
/// Sensor data
  sensorLaser = n.subscribe < sensor_msgs::LaserScan
      > ("/scan", 50, &walkRobotRos::laserSensorCallback, this);
/// Advertise Velocity
  velocity = n.advertise < geometry_msgs::Twist
      > ("/mobile_base/commands/velocity", 1000);
/// seeding random number generator
  srand(time(NULL));
}

/**
 * @brief Callback method that receives laser Scan information
 */
void walkRobotRos::laserSensorCallback(
    const sensor_msgs::LaserScan::ConstPtr& obsMsg) {
/// send laser scan message method of robotwalk class
  walk.laserSensorCallback(obsMsg);
}
/**
 * @brief Pusblishes msg to velocity of robot
 */
void walkRobotRos::robotWalkRos() {
  velocity.publish(walk.robotWalk());
}





