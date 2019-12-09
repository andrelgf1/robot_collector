/**
 /* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file walkRobot_test.cpp
 * @brief Source code for the implementation of walkRobot class
 * @author Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 */


#include <ros/ros.h>
#include <unistd.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include "walkRobotRos.hpp"


/**
 * @brief Test removeModels() of modelHandlingRos class
 *
 * @param[in]     walkRobotTest
 * @param[in]     TestObstacleFound
 *
 * @return     none
 */

TEST(walkRobotTest, TestObstacleFound) {
/// Creating Object of walkRobot
walkRobot rob;

/// setting obstacle equal true
rob.obstacle = true;

/// Calling method from walkRobot
auto msg = rob.robotWalk();

/// Expected linear velocity to avoid obstacle
EXPECT_EQ(0.4 , msg.angular.z);
}

/**
 * @brief Test removeModels() of modelHandlingRos class
 *
 * @param[in]     walkRobotTest
 * @param[in]     TestLinearVelocity
 *
 * @return     none
 */

TEST(walkRobotTest, TestLinearVelocity) {
/// Creating Object of walkRobot
walkRobot rob;

/// setting obstacle equal false
rob.obstacle = false;

/// setting linear velocity
rob.action = 0;

/// Calling method from walkRobot
auto msg = rob.robotWalk();

/// Expect linear velocity equal 1
EXPECT_EQ(1.0 , msg.linear.x);
}

/**
 * @brief Test removeModels() of modelHandlingRos class
 *
 * @param[in]     walkRobotTest
 * @param[in]     TestPositiveAngularVelocity
 *
 * @return     none
 */

TEST(walkRobotTest, TestPositiveAngularVelocity) {
/// Creating Object of walkRobot
walkRobot rob;

/// setting obstacle equal false
rob.obstacle = false;

/// setting action to positive velocity
rob.action = 1;

/// Calling method from walkRobot
auto msg = rob.robotWalk();

/// Expect linear velocity equal 0.5
EXPECT_EQ(0.5 , msg.angular.z);
}

/**
 * @brief Test removeModels() of modelHandlingRos class
 *
 * @param[in]     walkRobotTest
 * @param[in]     TestNegativeAngularVelocity
 *
 * @return     none
 */

TEST(walkRobotTest, TestNegativeAngularVelocity) {
/// Creating Object of walkRobot
walkRobot rob;

/// setting obstacle equal false
rob.obstacle = false;

/// setting action to negative velocity
rob.action = 2;

/// Calling method from walkRobot
auto msg = rob.robotWalk();

/// Expect linear velocity equal -0.5
EXPECT_EQ(-0.5 , msg.angular.z);
}

