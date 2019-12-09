/**
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file objectHandling.cpp
 * @brief Source code for object handling class
 * @author Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 */

#include "modelHandling.hpp"

/**
 * @brief Constructor for modelHandling
 */
modelHandling::modelHandling() {
}

/**
 * @brief Method returns msg with model info.
 */  
gazebo_msgs::SpawnModel modelHandling::getModel(std::string path,
  const std::string &id, float xPos, float yPos, float zPos) {
  /// Name of model
  std::string tagID = "ar_block" + id;

  /// Declaring gazebo_msgs
  gazebo_msgs::SpawnModel model;

  /// Retriving file
  std::ifstream file(path);
  std::string line;

  /// Giving xml lines of file to model
  while (!file.eof()) {
    std::getline(file, line);
    model.request.model_xml += line;
  }

  file.close();

  /// Setting necessary information to succesfully add model
  model.request.model_name = tagID;
  model.request.initial_pose.position.x = xPos;
  model.request.initial_pose.position.y = yPos;
  model.request.initial_pose.position.z = zPos;
  model.request.reference_frame = "world";

  return model;
}

/**
 * @brief Method returns a pair of gazebo_msgs for deleting a model.
 */
std::pair <gazebo_msgs::DeleteModelRequest, gazebo_msgs::DeleteModelResponse>
  modelHandling::getDeleteMsg(const std::string &id) {
  /// Name of model to be deleted
  std::string tagID = "ar_block" + id;

  /// Declaring gazebo_msgs to delete model
  gazebo_msgs::DeleteModelRequest deleteReq;
  gazebo_msgs::DeleteModelResponse deleteResp;
  deleteReq.model_name = tagID;

  /// Creating a pair for msgs above, to be given to ROSmodelHandling
  std::pair <gazebo_msgs::DeleteModelRequest, gazebo_msgs::DeleteModelResponse>
    msgPair(deleteReq, deleteResp);

  return msgPair;
}
