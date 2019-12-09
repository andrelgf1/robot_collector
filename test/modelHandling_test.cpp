/**
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file modelHandling_test.cpp
 * @brief Tests for modelHandling class
 * @author Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 */

#include <gtest/gtest.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <utility>
#include <string>
#include "modelHandling.hpp"

/**
 * @brief Test getModel() method of modelHandling class
 *
 * @param[in]     ModelHandlingTest
 * @param[in]     testGetModel
 *
 * @return     none
 */
TEST(ModelHandlingTest, testGetModel) {
  /// Declase a gazebo_msgs of type SpawnModel
  gazebo_msgs::SpawnModel testModel;
  std::string packagePath = ros::package::getPath("robot_collector");
  std::string path = packagePath + "/World/models/ar_block1/model.sdf";

  /// Retriving file
  std::ifstream file(path);
  std::string line;

  /// Giving xml lines of file to model
  while (!file.eof()) {
    std::getline(file, line);
    testModel.request.model_xml += line;
  }

  file.close();

  /// Setting info to testModel
  testModel.request.model_name = "ar_block1";
  testModel.request.initial_pose.position.x = 1.0;
  testModel.request.initial_pose.position.y = 2.0;
  testModel.request.initial_pose.position.z = 0.0;
  testModel.request.reference_frame = "world";

  /// modelHandling object
  modelHandling handling;

  /// Calling getModel
  gazebo_msgs::SpawnModel model = handling.getModel(path, "1", 1.0, 2.0, 0.0);

  /// Making sure the request sets correct parameters
  EXPECT_EQ(testModel.request.model_name, model.request.model_name);
  EXPECT_EQ(testModel.request.initial_pose.position.x, model.request.initial_pose.position.x);
  EXPECT_EQ(testModel.request.initial_pose.position.y, model.request.initial_pose.position.y);
  EXPECT_EQ(testModel.request.initial_pose.position.z, model.request.initial_pose.position.z);
  EXPECT_EQ(testModel.request.reference_frame, model.request.reference_frame);
}

/**
 * @brief Test getDeleteMsg() method of modelHandling class
 *
 * @param[in]     ModelHandlingTest
 * @param[in]     testGetDeleteMsg
 *
 * @return     none
 */
TEST(ModelHandlingTest, testGetDeleteMsg) {
  /// Declaring DeleteModel request and response for testing
  gazebo_msgs::DeleteModelRequest testReq;
  gazebo_msgs::DeleteModelResponse testResp;

  /// Setting info to testModel
  std::string tagName = "ar_block1";
  testReq.model_name = tagName;

  /// modelHandling object
  modelHandling handling;

  /// Call getDeleteMsg, which is a pair of response and request to delete a model
  std::pair <gazebo_msgs::DeleteModelRequest, gazebo_msgs::DeleteModelResponse>
    msgPair = handling.getDeleteMsg("1");

  EXPECT_EQ(msgPair.first.model_name, testReq.model_name);

  /// Expect false because service call hasn't been used
  EXPECT_FALSE(msgPair.second.success);
}
