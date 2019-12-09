/**
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file walkRobot.hpp
 * @brief Declaring class walkRobot
 * @author Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 */
#ifndef INCLUDE_WALK_ROBOT_WALKROBOT_HPP_
#define INCLUDE_WALK_ROBOT_WALKROBOT_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"


class walkRobot {
 private:
/// numberOfTimesPublished variable to the number of times a velovity is gonna be published
  int numberOfTimesPublished;

 public:
/// obstacle variable that handles the presence of an obstacle
  bool obstacle;
/// msg variable that handles robot velicities
  geometry_msgs::Twist msg;
/// action variable to enable  what action should be performed
  int action;
/**
 * @brief constructor of the walkRobot class
 * 
 * @param none
 * @return none
 */
  walkRobot();

/**
 * @brief Sensor callback function in order to subscribe in the laser sensor topic 
 * 
 * 
 * @param sensor_msgs::LaserScan
 * @return none
 */
  void laserSensorCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg);

/**
   * @brief Method to define robot speed in order to move Ramdomly while avoidind collision
   * @param none
   * @return geometry_msgs , In order to make the robot move
   */ 
  geometry_msgs::Twist robotWalk();
};

#endif  // INCLUDE_WALK_ROBOT_WALKROBOT_HPP_"
