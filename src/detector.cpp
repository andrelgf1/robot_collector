/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file detector.cpp
 * @brief Detects the closest AR tag.
 * @author Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Andre Gomes, Ryan Cunningham
 */

#include "detector.hpp"

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <math.h>

Detector::Detector(const std::vector<std::string>& validMarkerIds): validMarkerIds(validMarkerIds) {}

void Detector::detect(const ar_track_alvar_msgs::AlvarMarkers& msg) {
  /// remove existing markers
  markers.clear();

  /// add new observed markers
  for (auto marker : msg.markers) {
    if (isMarkerIdValid(std::to_string(marker.id))) {
      markers.push_back(marker);
    }
  }

  if (markers.size() > 1) {
    /// sort by the position
    std::sort(markers.begin(), markers.end(),
              [](const ar_track_alvar_msgs::AlvarMarker& marker1, const ar_track_alvar_msgs::AlvarMarker& marker2) {
                /// calculate the euclidean distance of each marker
                double marker1Distance = sqrt(pow(marker1.pose.pose.position.x, 2)
                    + pow(marker1.pose.pose.position.y, 2));
                double marker2Distance = sqrt(pow(marker2.pose.pose.position.x, 2)
                    + pow(marker2.pose.pose.position.y, 2));

                /// sort so that the markers in the vector are closer than ones after it in index position
                return marker1Distance < marker2Distance;
              });
  }
}

bool Detector::isMarkerDetected() {
  /// check if the markers list is empty
  return !markers.empty();
}

ar_track_alvar_msgs::AlvarMarker Detector::getClosestMarker() {
  /// return the first marker (will be closest since the values are sorted)
  return markers[0];
}

bool Detector::isMarkerIdValid(const std::string& markerId) {
  /// check if the markerId is found in the list of validMarkerIds
  return std::find(validMarkerIds.begin(), validMarkerIds.end(), markerId) != validMarkerIds.end();
}
