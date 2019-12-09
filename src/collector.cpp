/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file collector.cpp
 * @brief Navigates to the marker that is to be collected.
 * @author Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Andre Gomes, Ryan Cunningham
 */

#include "collector.hpp"

#include <actionlib/client/simple_action_client.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/tf.h>
#include <string>

#include "modelHandling.hpp"

Collector::Collector(double collectionRange): collectionRange(collectionRange)  {}

move_base_msgs::MoveBaseGoal Collector::createMoveBaseGoal(const ar_track_alvar_msgs::AlvarMarker& marker, const std::string& frameId) {
  /// create the goal
  move_base_msgs::MoveBaseGoal goal;

  /// set the frame id and time
  goal.target_pose.header.frame_id = frameId;
  goal.target_pose.header.stamp = ros::Time::now();

  /// get the translated point
  std::pair<double, double> adjustedPoint = translatePointToOrigin(
      marker.pose.pose.position.x, marker.pose.pose.position.y
      );

  /// set the position
  goal.target_pose.pose.position.x = adjustedPoint.first;
  goal.target_pose.pose.position.y = adjustedPoint.second;
  goal.target_pose.pose.position.z = 0;

  /// set the orientation
  goal.target_pose.pose.orientation.x = 0;
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = 0;
  goal.target_pose.pose.orientation.w = 1;

  /// return the goal
  return goal;
}

std::pair<double, double> Collector::translatePointToOrigin(double x, double y) {
  /// get the distance from the robot
  double distance = sqrt(pow(x, 2) + pow(y, 2));

  /// get the angle from the robot
  double angle = atan2(x, y);

  /// get the x and y points collectionRange away from the robot
  double xAdjusted = sin(angle) * (distance - collectionRange);
  double yAdjusted = cos(angle) * (distance - collectionRange);

  /// return the new point
  return std::make_pair(xAdjusted, yAdjusted);
}
