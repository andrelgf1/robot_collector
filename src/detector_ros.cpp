/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file detector_ros.cpp
 * @brief ROS wrapper for the DetectorRos class.
 * @author Pablo Sanhueza
 * @author Ryan Cunningham
 * @author Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Andre Gomes, Ryan Cunningham
 */

#include "detector_ros.hpp"

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <math.h>

DetectorRos::DetectorRos(const std::vector<std::string>& validMarkerIds): detector(validMarkerIds) {
  /// initialize the subscriber
  arTrackSub = nh.subscribe("ar_pose_marker", 1000, &DetectorRos::detect, this);
}

void DetectorRos::detect(const ar_track_alvar_msgs::AlvarMarkers& msg) {
  /// call the detect method
  detector.detect(msg);
}

bool DetectorRos::isMarkerDetected() {
  return detector.isMarkerDetected();
}

ar_track_alvar_msgs::AlvarMarker DetectorRos::getClosestMarker() {
  return detector.getClosestMarker();
}
