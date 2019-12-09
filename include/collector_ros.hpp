/**
 * @file collector_ros.hpp
 * @brief ROS wrapper for the Collector class.
 * @author Pablo Sanhueza
 * @author Ryan Cunningham
 * @author Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Andre Gomes, Ryan Cunningham
 *
 */

#ifndef INCLUDE_COLLECTOR_ROS_HPP_
#define INCLUDE_COLLECTOR_ROS_HPP_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <string>

#include "modelHandling.hpp"

#include "collector.hpp"

class CollectorRos {
 public:
  /**
   * @brief Initializes the CollectorRos object.
   * @param collectionRange the range to the robot to collect
   * @return none
   */
  CollectorRos(const std::string& frameId, double collectionRange);
  /**
   * @brief Navigates to the marker to collect it.
   * @param marker the marker to collect
   * @return true if the marker is reached, false otherwise
   */
  bool collect(const ar_track_alvar_msgs::AlvarMarker& marker);
 private:
  /// the coordinate from the goal position is in
  std::string frameId;
  /// the move base action client
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseClient;
  /// the collector object
  Collector collector;
};

#endif // INCLUDE_COLLECTOR_ROS_HPP_
