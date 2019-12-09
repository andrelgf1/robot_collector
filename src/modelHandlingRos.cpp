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

#include "modelHandlingRos.hpp"

/**
 * @brief Constructor for modelHandlingRos
 */
modelHandlingRos::modelHandlingRos(const std::string rosPackage):
  rosPackage_(rosPackage) {
  /// Service from gazebo_ros to spawn sdf models
  spawnClient = n.serviceClient<gazebo_msgs::SpawnModel>
    ("/gazebo/spawn_sdf_model");

  /// Service from gazebo_ros to delete models from gazebo world
  deleteClient = n.serviceClient<gazebo_msgs::DeleteModel>
    ("/gazebo/delete_model");

  /// Subscribe to gazebo/model_states
  world = n.subscribe("/gazebo/model_states", 50,
    &modelHandlingRos::modelStatesCallback, this);

  ROS_INFO_STREAM("modelHandlingRos started");
}

/**
 * @brief Adds models to Gazebo world
 */  
void modelHandlingRos::addModels(const std::string id, float xPos, float yPos,
float zPos) {
  ROS_INFO_STREAM("ROSmodelHandling::addModels");

  if (!spawnClient.waitForExistence(ros::Duration(2.0))) {
    ROS_FATAL_STREAM("Unable to locate service '" <<
      spawnClient.getService() << "'");
  }

  modelPath(id);
  model = handling.getModel(path, id, xPos, yPos, zPos);
  /// Call the service to add model
  spawnClient.call(model);
  ROS_INFO_STREAM("Model Added - " << id);
}

/**
 * @brief Removes models from Gazebo world
 */
void modelHandlingRos::removeModels(const std::string id) {
  if (!deleteClient.waitForExistence(ros::Duration(2.0))) {
    ROS_FATAL_STREAM("Unable to locate service '" <<
      spawnClient.getService() << "'");
  }
  /// Retrivieng gazebo_msgs necessary to delete model
  deleteMsg = handling.getDeleteMsg(id);

  /// Calling the service to delete model
  deleteClient.call(deleteMsg.first, deleteMsg.second);
  ROS_INFO_STREAM("Model Deleted - " << id);
}

/**
 * @brief Adds Models randomly in a defined Gazebo world
 */
void modelHandlingRos::randModels(std::vector<std::string> modelNames,
  float width, float length, std::vector<float> centerPoint) {
  /// Initializing randCoord object
  randCoord randomize(width, length, centerPoint);

  ROS_INFO_STREAM("modelHandlingRos::randModels");

  /// Declaring variables for x and y rand coords
  float xRand;
  float yRand;

  /// Constant z
  const float z = .22;

  ROS_INFO_STREAM("Adding models randomly to map");

  /// Make sure that Model States msgs are active
  while (models.size() == 0) {
    ros::spinOnce();
  }

  /// Checking if random coords are close to other models in map
  for (auto name : modelNames) {
    for (auto modelPos : models) {
      xRand = randomize.randX();
      yRand = randomize.randY();

      while ((pow((xRand-modelPos.position.x), 2) + pow((yRand -
        modelPos.position.y), 2)) <= pow(0.5, 2)) {
        xRand = randomize.randX();
        yRand = randomize.randY();
       }
    }

    /// Adding model
    addModels(name, xRand, yRand, z);
  }
}

/**
 * @brief Return the path of the model to be used
 */
void modelHandlingRos::modelPath(std::string id) {
  /// Name of model
  std::string tagID = "ar_block" + id;

  /// Getting package of path
  std::string packagePath = ros::package::getPath(rosPackage_);

  /// Relative path to obtain file
  std::string relativePath = "/World/models/" + tagID + "/model.sdf";

  /// Full path to model file
  path = packagePath + relativePath;
}

/**
 * @brief Call back method for ModelStates Topic
 */
void modelHandlingRos::modelStatesCallback
  (const gazebo_msgs::ModelStates &msg) {
  /// Clear vector
  models.clear();

  /// Adding poses of objects in map
  for (auto poses : msg.pose) {
    models.push_back(poses);
  }
}

/**
 * @brief Getter for Spawn Client
 */
ros::ServiceClient modelHandlingRos::getSpawnClient() {
  return spawnClient;
}

/**
 * @brief Getter for Delete Client
 */
ros::ServiceClient modelHandlingRos::getDeleteClient() {
  return deleteClient;
}


