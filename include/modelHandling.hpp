/**
 * /* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file objectHandling.hpp
 * @brief Header file for object handling class
 * @author Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 */

#ifndef INCLUDE_MODELHANDLING_HPP_
#define INCLUDE_MODELHANDLING_HPP_

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <utility>
#include <string>
#include <random>

/**
 * @brief Class for sending messages to add or delete models.
 */
class modelHandling {
 public:
  /**
   * @brief Constructor for modelHandling
   */
  modelHandling();

  /**
   * @brief Method returns msg with model info.
   * @param path Path to model file
   * @param id Tag id
   * @param xPos x position of model
   * @param yPos y position of model
   * @param zPos z position of model
   * @return gazebo_msgs::SpawnModel returns a message of type SpawnModel
   */  
  gazebo_msgs::SpawnModel getModel(std::string path, const std::string id,
    float xPos, float yPos, float zPos);

  /**
   * @brief Method returns a pair of gazebo_msgs for deleting a model.
   * @param id Tag id as a string
   * @return std::pair <gazebo_msgs::DeleteModelRequest, gazebo_msgs::DeleteModelResponse> returns a pair of gazebo_msgs to delete model
   */  
  std::pair <gazebo_msgs::DeleteModelRequest, gazebo_msgs::DeleteModelResponse>
    getDeleteMsg(const std::string id);
};

#endif  // INCLUDE_MODELHANDLING_HPP_
