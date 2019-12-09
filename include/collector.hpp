/**
 * @file collector.hpp
 * @brief Navigates to the marker that is to be collected.
 * @author Pablo Sanhueza
 * @author Ryan Cunningham
 * @author Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Andre Gomes, Ryan Cunningham
 *
 */

#ifndef INCLUDE_COLLECTOR_HPP_
#define INCLUDE_COLLECTOR_HPP_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <string>
#include <utility>

#include "modelHandling.hpp"

class Collector {
 public:
  /**
   * @brief Initializes the Collector object.
   * @param collectionRange the range to the robot to collect 
   * @return none
   */
  explicit Collector(double collectionRange);
  /**
   * @brief Creates a move base goal message.
   * @param marker the marker to set as the goal
   * @param frameId the coordinate frame to set the position
   * @return goal message
   */
  move_base_msgs::MoveBaseGoal createMoveBaseGoal(const ar_track_alvar_msgs::AlvarMarker& marker, const std::string& frameId);
  /**
   * @brief Translates a point to the origin by collectionRange.
   * @param x the point x value
   * @param y the point y value
   * @return the translated point
   */
  std::pair<double, double> translatePointToOrigin(double x, double y);
 private:
  /// collection range to the robot to collect
  double collectionRange;
};

#endif // INCLUDE_COLLECTOR_HPP_
