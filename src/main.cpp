/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file main.cpp
 * @brief Demo program for the collector robot.
 * @author Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Andre Gomes, Ryan Cunningham
 */

#include <ros/ros.h>

#include "collector_ros.hpp"
#include "detector_ros.hpp"
#include "modelHandlingRos.hpp"
#include "walkRobotRos.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_collector_node");

  modelHandlingRos models("robot_collector");
  std::vector<std::string> tag_names = {"0", "1", "2", "3"};
  std::vector<float> midPoint = {0, 0};
  float width = 5.0;
  float length = 8.0;
  models.randModels(tag_names, width, length, midPoint);

  CollectorRos collector("base_link", 0.5);

  DetectorRos detector(tag_names);

  walkRobotRos walkRobot;

  ros::Rate rate(20);

  while (ros::ok()) {
    if (detector.isMarkerDetected()) {
      ar_track_alvar_msgs::AlvarMarker marker = detector.getClosestMarker();
      bool collected = collector.collect(marker);
      if (collected) {
        models.removeModels(std::to_string(marker.id));
      }
    }

    walkRobot.robotWalkRos();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
