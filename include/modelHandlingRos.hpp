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

#ifndef INCLUDE_MODELHANDLINGROS_HPP_
#define INCLUDE_MODELHANDLINGROS_HPP_

#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <utility>
#include "ros/package.h"
#include "randCoord.hpp"
#include "modelHandling.hpp"

/**
 * @brief Class for adding models into a Gazebo world.
 */
class modelHandlingRos {
 public:
  /// ROS Package of where the models to be added
  std::string rosPackage_;

  /// Path to where to sdf model
  std::string path;

  /// Gazebo Msg Model
  gazebo_msgs::SpawnModel model;

  /// Pair of Delete Model msgs
  std::pair <gazebo_msgs::DeleteModelRequest, gazebo_msgs::DeleteModelResponse>
    deleteMsg;

  /**
   * @brief Constructor for ROSmodelHandling
   */
  explicit modelHandlingRos(const std::string &rosPackage);

  /**
   * @brief Adds models to Gazebo world
   * @param id Tag id
   * @param xPos x position of model
   * @param yPos y position of model
   * @param zPos z position of model
   * @return none
   */  
  void addModels(const std::string &id, float xPos, float yPos, float zPos);

  /**
   * @brief Removes blocks from Gazebo world
   * @param id Tag id as a string
   * @return none
   */  
  void removeModels(const std::string &id);

  /**
   * @brief Adds Models randomly in a defined Gazebo world
   * @param modelNames vector of model names as strings
   * @param width Width of the Gazebo world
   * @param height Height of Gazebo world
   * @param worldMidPoint Center of Gazebo world
   * @return none
   */  
  void randModels(std::vector<std::string> modelNames, float width,
    float length, std::vector<float> centerPoint);

  /**
   * @brief Getter for Spawn ServiceClient
   * @param none
   * @return ros::ServiceClient
   */
  ros::ServiceClient getSpawnClient();

  /**
   * @brief Getter for Delete ServiceClient
   * @param none
   * @return ros::ServiceClient
   */
  ros::ServiceClient getDeleteClient();

 private:
  /// modelHandling object
  modelHandling handling;

  /// Node handle object
  ros::NodeHandle n;

  /// Clients & Subscriber
  ros::ServiceClient spawnClient;
  ros::ServiceClient deleteClient;
  ros::Subscriber world;

  /// Private member for models
  std::vector<geometry_msgs::Pose> models;

  /**
   * @brief Return the path of the model to be used
   * @param id Tag id as a string
   * @return none
   */  
  void modelPath(std::string &id);

  /**
   * @brief Call back method for ModelStates Topic
   * @param id msg ModelState message with all poses of objects in gazebo map
   * @return none
   */  
  void modelStatesCallback(const gazebo_msgs::ModelStates &msg);
};

#endif  // INCLUDE_MODELHANDLINGROS_HPP_
