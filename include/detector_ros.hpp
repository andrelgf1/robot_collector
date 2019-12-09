/**
 * @file detector.hpp
 * @brief Detects the closest AR tag.
 * @author Pablo Sanhueza
 * @author Ryan Cunningham
 * @author Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Andre Gomes, Ryan Cunningham
 *
 */

#ifndef INCLUDE_DETECTOR_HPP_
#define INCLUDE_DETECTOR_HPP_

#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>

#include <string>
#include <vector>

/**
 * @brief Detects AR tags.
 */
class Detector {
 public:
  /**
   * @brief Initialize Detector object.
   * @param validMarkerIds valid marker ids to detect
   * @return none
   */
  explicit Detector(const std::vector<std::string>& validMarkerIds);
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
  /**
   * @brief Check if a marker id is valid.
   * @param markerId the marker id to check
   * @return true is marker is valid, false otherwise
   */
  bool isMarkerIdValid(const std::string& markerId);
 private:
  /// valid marker ids
  std::vector<std::string> validMarkerIds;
  /// detected markers
  std::vector<ar_track_alvar_msgs::AlvarMarker> markers;
};

#endif // INCLUDE_DETECTOR_HPP_
