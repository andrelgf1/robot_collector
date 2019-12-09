/**
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file walkRobotRos.hpp
 * @brief Declaring class walkRobotRos
 * @author Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 */
#ifndef INCLUDE_WALK_ROBOT_WALKROBOTROS_HPP_
#define INCLUDE_WALK_ROBOT_WALKROBOTROS_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"

#include "walkRobot.hpp"

class walkRobotRos {
 private:
  /// Node handle object
  ros::NodeHandle n;

  /// Subscribe to laser scan topic
  ros::Subscriber sensorLaser;

  /// Publisher object. This will publish to robot velocity
  ros::Publisher velocity;

 public:
  /// Object of walk robot class
  walkRobot walk;
 /**
 * @brief constructor of the walkRobotRos class
 * 
 * @param none
 * @return none
 */
  walkRobotRos();

/**
 * @brief Sensor callback function in order to subscribe in the laser sensor topic and
 * set the atribute obstacle in order to inform the presence or not of obstacles.
 * 
 * @param sensor_msgs::LaserScan::ConstPtr
 * @return none
 */
  void laserSensorCallback(const sensor_msgs::LaserScan::ConstPtr&);

/**
  * @brief Publishes msg to robot velocity
  * @param none
  * @return none
  */ 
  void robotWalkRos();
};

#endif  // INCLUDE_WALK_ROBOT_WALKROBOTROS_HPP_"
