/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file collector_ros.cpp
 * @brief ROS wrapper for the Collector class.
 * @author Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Andre Gomes, Ryan Cunningham
 */

#include "collector_ros.hpp"

#include <actionlib/client/simple_action_client.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/tf.h>
#include <string>

#include "modelHandling.hpp"

CollectorRos::CollectorRos(const std::string& frameId, double collectionRange): frameId(frameId), moveBaseClient("move_base"), collector(collectionRange) {
  while (!moveBaseClient.waitForServer(ros::Duration(5.0))) {
    ROS_INFO_STREAM("Waiting for the move_base action server to come up");
  }
}

bool CollectorRos::collect(const ar_track_alvar_msgs::AlvarMarker& marker) {
  /// get the move base goal
  move_base_msgs::MoveBaseGoal goal = collector.createMoveBaseGoal(marker, frameId);

  /// send the goal and wait for the robot to move
  moveBaseClient.sendGoal(goal);
  moveBaseClient.waitForResult();

  if (moveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO_STREAM("Reached marker!");

    return true;
  } else {
    ROS_WARN_STREAM("Did not reach marker!");

    return false;
  }
}
