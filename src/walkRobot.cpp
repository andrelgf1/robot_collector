/**
 /* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file walkRobot.cpp
 * @brief Source code for the implementation of walkRobot class
 * @author Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 */

#include "walkRobot.hpp"

#include <stdlib.h>
#include <ros/ros.h>
#include <time.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"

walkRobot::walkRobot() {
/// Initializing obstacle variable to false
  obstacle = false;

/// Initializing numberOfTimesPublished variable to 0
  numberOfTimesPublished = 0;

/// Initializing action variable to 0
  action = 0;

  srand(time(NULL));
}

/**
 * @brief Method that calls back LaserScan
 */
void walkRobot::laserSensorCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg) {
/// checking if laser found any obstacle
  for (auto i : scanMsg->ranges) {
    if (i < 1) {
      obstacle = true;
      return;
    }
  }
/// if no obstacle was found set obstacle to false
  obstacle = false;
}

/**
 * @brief Method that moves robot randomly while avoiding obstacles
 */
geometry_msgs::Twist walkRobot::robotWalk() {
/// check obstacle variable in order to ser the velocity
  if (obstacle == false) {
/// Generarate a random number to variables that controls the number of times the velocity published and which action shoul be done
    if (numberOfTimesPublished > 10) {
      numberOfTimesPublished = rand() % 9;
      action = rand() % 3;
    }
/// Define the velocity that should be appiled based on the random variable information
    if (action == 0) {
      msg.linear.x = 1.0;
      msg.angular.z = 0;
      std::cout << "linear " << action << " " << numberOfTimesPublished << "\n";
    } else if (action == 1) {
      msg.linear.x = 0;
      msg.angular.z = 0.5;
      std::cout << "angular right " << action << " " << numberOfTimesPublished
          << "\n";
    } else {
      msg.linear.x = 0;
      msg.angular.z = -0.5;
      std::cout << "angular left " << action << " " << numberOfTimesPublished
          << "\n";
    }
    // ROS_INFO_STREAM("Setting linear velocity");
    // ROS_INFO_STREAM("Number of obstacles avoided "<< robot.getAvoided());
  } else {
/// if the robot detects an obstacle set angular velocity
    msg.angular.z = 0.4;
    ROS_INFO_STREAM("Setting angular velocity to Avoid Obstacle.");
/// Make sure the robot does not have any linear velocity
    msg.linear.x = 0.0;
/// Increment the number of obstacles only if the flag equal true
  }
/// Incrementing variable numberOfTimesPublished
  numberOfTimesPublished++;
/// Calling Function to publish the velocity
  return msg;
}




