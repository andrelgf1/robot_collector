/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file detector_ros_test.cpp
 * @brief Unit Test for DetectorRos class.
 * @author Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Andre Gomes, Ryan Cunningham
 */

#include "detector_ros.hpp"

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

/**
 * @brief Test that no markers are detected when a message is published that has no markers.
 * @param[in] DetectorRosTest
 * @param[in] testNoDetectedMarkers
 * @return none
 */
TEST(DetectorRosTest, testNoDetectedMarkers) {
  ros::NodeHandle nh;
  /// create a publisher to replicate the ar_track_alvar node
  ros::Publisher pub = nh.advertise<ar_track_alvar_msgs::AlvarMarkers>("ar_pose_marker", 1000);

  std::vector<std::string> validMarkerIds = {"0", "1"};
  DetectorRos detector(validMarkerIds);

  /// create a message with no markers
  ar_track_alvar_msgs::AlvarMarkers msg;
  msg.markers = {};

  ros::Rate rate(5);
  int count = 0;

  /// need to wait since there is a delay with ros in unit tests
  while (ros::ok() && count < 2) {
    pub.publish(msg);
    ros::spinOnce();

    rate.sleep();
    count++;
  }

  /// verify that no markers are detected
  EXPECT_FALSE(detector.isMarkerDetected());
}

/**
 * @brief Test that markers are detected when a message is published that contains markers.
 * @param[in] DetectorRosTest
 * @param[in] testDetectedMarkers
 * @return none
 */
TEST(DetectorRosTest, testDetectedMarkers) {
  ros::NodeHandle nh;
  /// create a publisher to replicate the ar_track_alvar node
  ros::Publisher pub = nh.advertise<ar_track_alvar_msgs::AlvarMarkers>("ar_pose_marker", 1000);

  std::vector<std::string> validMarkerIds = {"0", "1"};
  DetectorRos detector(validMarkerIds);

  /// create a message with a marker
  ar_track_alvar_msgs::AlvarMarkers msg;

  ar_track_alvar_msgs::AlvarMarker marker;
  marker.id = 0;
  marker.pose.pose.position.x = 1;
  marker.pose.pose.position.y = 1;
  marker.pose.pose.position.z = 1;
  marker.pose.pose.orientation.x = 1;
  marker.pose.pose.orientation.y = 1;
  marker.pose.pose.orientation.z = 1;

  msg.markers = { marker };

  ros::Rate rate(5);
  int count = 0;

  /// need to wait since there is a delay with ros in unit tests
  while (ros::ok() && count < 2) {
    pub.publish(msg);
    ros::spinOnce();

    rate.sleep();
    count++;
  }

  /// verify that the marker is detected
  EXPECT_TRUE(detector.isMarkerDetected());
}

/**
 * @brief Test that the detector returns the closest of multiple markers.
 * @param[in] DetectorRosTest
 * @param[in] testIsCloserThan
 * @return none
 */
TEST(DetectorRosTest, testIsCloserThan) {
  ros::NodeHandle nh;
  /// create a publisher to replicate the ar_track_alvar node
  ros::Publisher pub = nh.advertise<ar_track_alvar_msgs::AlvarMarkers>("ar_pose_marker", 1000);

  std::vector<std::string> validMarkerIds = {"0", "1"};
  DetectorRos detector(validMarkerIds);

  /// create a message with two markers
  ar_track_alvar_msgs::AlvarMarkers msg;

  /// the first marker is further away from the robot
  ar_track_alvar_msgs::AlvarMarker marker1;
  marker1.id = 0;
  marker1.pose.pose.position.x = 1;
  marker1.pose.pose.position.y = 1;
  marker1.pose.pose.position.z = 1;
  marker1.pose.pose.orientation.x = 1;
  marker1.pose.pose.orientation.y = 1;
  marker1.pose.pose.orientation.z = 1;

  /// the second marker is closer to the robot
  ar_track_alvar_msgs::AlvarMarker marker2;
  marker2.id = 1;
  marker2.pose.pose.position.x = 0.5;
  marker2.pose.pose.position.y = 0.5;
  marker2.pose.pose.position.z = 0.5;
  marker2.pose.pose.orientation.x = 1;
  marker2.pose.pose.orientation.y = 1;
  marker2.pose.pose.orientation.z = 1;

  msg.markers = { marker1, marker2 };

  ros::Rate rate(5);
  int count = 0;

  /// need to wait since there is a delay with ros in unit tests
  while (ros::ok() && count < 2) {
    pub.publish(msg);
    ros::spinOnce();

    rate.sleep();
    count++;
  }

  /// verify that the marker is detected
  EXPECT_TRUE(detector.isMarkerDetected());
  /// verify that the correct marker is the closest
  EXPECT_EQ(1, detector.getClosestMarker().id);
}
