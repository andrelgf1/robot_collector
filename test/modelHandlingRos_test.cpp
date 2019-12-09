/**
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file modelHandlingRos_test.cpp
 * @brief Test modelHandlingRos class
 * @author Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ModelStates.h>
#include "modelHandlingRos.hpp"

/**
 * @brief Test Constructor of modelHandlingRos class
 *
 * @param[in]     ModelHandlingRosTest
 * @param[in]     testConstructorServiceClients
 *
 * @return     none
 */
TEST(ModelHandlingRosTest, testConstructorServiceClients) {
  /// Services used
  std::string spawnService = "/gazebo/spawn_sdf_model";
  std::string deleteService = "/gazebo/delete_model";

  /// modelHandlingRos object - pass ros package name
  modelHandlingRos modelROS("robot_collector");

  /// Get Service Clients
  ros::ServiceClient testSpawnClient = modelROS.getSpawnClient();
  ros::ServiceClient testDeleteClient = modelROS.getDeleteClient();

  /// Make sure that ServiceClient Exists
  if (!testSpawnClient.waitForExistence(ros::Duration(-1.0))) {
    ROS_FATAL_STREAM("Unable to locate service '" <<
      testSpawnClient.getService() << "'");
  }

  if (!testDeleteClient.waitForExistence(ros::Duration(-1.0))) {
    ROS_FATAL_STREAM("Unable to locate service '" <<
      testDeleteClient.getService() << "'");
  }

  /// Run tests
  EXPECT_TRUE(testSpawnClient.exists());
  EXPECT_TRUE(testDeleteClient.exists());
  EXPECT_EQ(spawnService, testSpawnClient.getService());
  EXPECT_EQ(deleteService, testDeleteClient.getService());
}

/**
 * @brief Test addModels() method of modelHandlingRos class
 *
 * @param[in]     ModelHandlingRosTest
 * @param[in]     testAddModels
 *
 * @return     none
 */
TEST(ModelHandlingRosTest, testAddModels) {
  /// modelHandlingRos object
  modelHandlingRos modelROS("robot_collector");

  /// parameters of model
  std::string id = "1";
  float xPos = 1.0;
  float yPos = 1.0;
  float zPos = 1.0;

  /// Add models to world
  modelROS.addModels(id, xPos, yPos, zPos);

  /// Get public variable from modelHandlingClass
  gazebo_msgs::SpawnModel testModel = modelROS.model;

  /// Run test
  EXPECT_TRUE(testModel.response.success);
}

/**
 * @brief Test removeModels() of modelHandlingRos class
 *
 * @param[in]     ModelHandlingRosTest
 * @param[in]     testRemoveModels
 *
 * @return     none
 */
TEST(ModelHandlingRosTest, testRemoveModels) {
  /// modelHandlingRos object
  modelHandlingRos modelROS("robot_collector");

  /// Model id to be deleted
  std::string id = "1";

  /// Remove model from world
  modelROS.removeModels(id);

  /// Get Delete ServiceClient
  ros::ServiceClient testDeleteClient = modelROS.getDeleteClient();
  std::pair <gazebo_msgs::DeleteModelRequest, gazebo_msgs::DeleteModelResponse>
    deleteMsgTest = modelROS.deleteMsg;

  /// Run test
  EXPECT_TRUE(deleteMsgTest.second.success);
}
