/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file detector_test.cpp
 * @brief Unit Test for Detector class.
 * @author Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Andre Gomes, Ryan Cunningham
 */

#include "detector.hpp"

#include <gtest/gtest.h>

/**
 * @brief Test that no markers are detected from a message with no markers.
 * @param[in] DetectorTest
 * @param[in] testNoDetectedMarkers
 * @return none
 */
TEST(DetectorTest, testNoDetectedMarkers) {
  std::vector<std::string> validMarkerIds = {"0", "1"};
  Detector detector = Detector(validMarkerIds);

  /// create a message with no markers
  ar_track_alvar_msgs::AlvarMarkers msg;
  msg.markers = {};

  detector.detect(msg);

  /// verify no markers are detected
  EXPECT_FALSE(detector.isMarkerDetected());
}

/**
 * @brief Test that markers are detected from a message that contains markers.
 * @param[in] DetectorTest
 * @param[in] testDetectedMarkers
 * @return none
 */
TEST(DetectorTest, testDetectedMarkers) {
  std::vector<std::string> validMarkerIds = {"0", "1"};
  Detector detector = Detector(validMarkerIds);

  /// create a message with 1 marker
  ar_track_alvar_msgs::AlvarMarkers msg;

  /// create the marker message
  ar_track_alvar_msgs::AlvarMarker marker;
  marker.id = 0;
  marker.pose.pose.position.x = 1;
  marker.pose.pose.position.y = 1;
  marker.pose.pose.position.z = 1;
  marker.pose.pose.orientation.x = 1;
  marker.pose.pose.orientation.y = 1;
  marker.pose.pose.orientation.z = 1;

  msg.markers = { marker };

  detector.detect(msg);

  /// verify the marker is detected
  EXPECT_TRUE(detector.isMarkerDetected());
}

/**
 * @brief Test that the detector returns the closest of multiple markers.
 * @param[in] DetectorTest
 * @param[in] testIsCloserThan
 * @return none
 */
TEST(DetectorTest, testIsCloserThan) {
  std::vector<std::string> validMarkerIds = {"0", "1"};
  Detector detector = Detector(validMarkerIds);

  /// create a message with two markers
  ar_track_alvar_msgs::AlvarMarkers msg;

  /// the first message will be further away from the robot
  ar_track_alvar_msgs::AlvarMarker marker1;
  marker1.id = 0;
  marker1.pose.pose.position.x = 1;
  marker1.pose.pose.position.y = 1;
  marker1.pose.pose.position.z = 1;
  marker1.pose.pose.orientation.x = 1;
  marker1.pose.pose.orientation.y = 1;
  marker1.pose.pose.orientation.z = 1;

  /// the second message will be closer to the robot
  ar_track_alvar_msgs::AlvarMarker marker2;
  marker2.id = 1;
  marker2.pose.pose.position.x = 0.5;
  marker2.pose.pose.position.y = 0.5;
  marker2.pose.pose.position.z = 0.5;
  marker2.pose.pose.orientation.x = 1;
  marker2.pose.pose.orientation.y = 1;
  marker2.pose.pose.orientation.z = 1;

  msg.markers = { marker1, marker2 };

  detector.detect(msg);

  /// verify that the markers are detected
  EXPECT_TRUE(detector.isMarkerDetected());
  /// verify that the correct marker is the closest
  EXPECT_EQ(1, detector.getClosestMarker().id);
}

/**
 * @brief Test that a marker id is in the list of valid marker ids.
 * @param[in] DetectorTest
 * @param[in] testIsMarkerIdValid
 * @return none
 */
TEST(DetectorTest, testIsMarkerIdValid) {
  /// create valid marker ids
  std::vector<std::string> validMarkerIds = {"0", "1"};
  Detector detector = Detector(validMarkerIds);

  /// verify that an id in the list of valid ids is valid
  EXPECT_TRUE(detector.isMarkerIdValid("1"));
  /// verify that an id not in the list of valid ids is invalid
  EXPECT_FALSE(detector.isMarkerIdValid("2"));
}
