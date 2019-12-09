/**
 * @file detector_ros.hpp
 * @brief ROS wrapper for he DetectorRos class.
 * @author Pablo Sanhueza
 * @author Ryan Cunningham
 * @author Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Andre Gomes, Ryan Cunningham
 *
 */

#ifndef INCLUDE_DETECTOR_ROS_HPP_
#define INCLUDE_DETECTOR_ROS_HPP_

#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>

#include <string>
#include <vector>

#include "detector.hpp"

/**
 * @brief Detects AR tags.
 */
class DetectorRos {
 public:
  /**
   * @brief Initialize Detector object.
   * @param validMarkerIds valid marker ids to detect
   * @return none
   */
  explicit DetectorRos(const std::vector<std::string>& validMarkerIds);
  /**
   * @brief Detect markers.
   * @param msg markers message
   * @return void
   */
  void detect(const ar_track_alvar_msgs::AlvarMarkers& msg);
  /**
   * @brief Check whether a marker has been detected.
   * @param none
   * @return true if a marker is detected, false otherwise
   */
  bool isMarkerDetected();
  /**
   * @brief Get the marker closest to the robot.
   * @param none
   * @return the closest marker
   */
  ar_track_alvar_msgs::AlvarMarker getClosestMarker();

 private:
  /// node handle for subscribing
  ros::NodeHandle nh;
  /// subscription to the ar_pose_marker topic
  ros::Subscriber arTrackSub;
  /// marker detector
  Detector detector;
};

#endif // INCLUDE_DETECTOR_ROS_HPP_
