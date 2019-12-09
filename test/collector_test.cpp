/**
 * @file collector_test.cpp
 * @copyright 2019
 *
 */

#include "collector.hpp"

#include <gtest/gtest.h>

TEST(CollectorTest, testCreateMoveBaseGoal) {
  ros::Time::init();

  Collector collector(0.4);

  ar_track_alvar_msgs::AlvarMarker marker;

  marker.pose.pose.position.x = 4;
  marker.pose.pose.position.y = 5;
  marker.pose.pose.position.z = 1;

  marker.pose.pose.orientation.x = 1;
  marker.pose.pose.orientation.y = 1;
  marker.pose.pose.orientation.z = 1;
  marker.pose.pose.orientation.w = 0;

  move_base_msgs::MoveBaseGoal goal = collector.createMoveBaseGoal(marker, "base_link");

  EXPECT_TRUE(marker.pose.pose.position.x > goal.target_pose.pose.position.x);
  EXPECT_TRUE(marker.pose.pose.position.y > goal.target_pose.pose.position.y);
  EXPECT_EQ(0, goal.target_pose.pose.position.z);

  EXPECT_EQ(0, goal.target_pose.pose.orientation.x);
  EXPECT_EQ(0, goal.target_pose.pose.orientation.y);
  EXPECT_EQ(0, goal.target_pose.pose.orientation.z);
  EXPECT_EQ(1, goal.target_pose.pose.orientation.w);
}

TEST(CollectorTest, testTranslatePointToOrigin) {
  double collectionRange = 0.4;
  Collector collector(collectionRange);

  double x = 4;
  double y = 1;

  std::pair<double, double> point = collector.translatePointToOrigin(x, y);

  double distance = sqrt(pow(x - point.first, 2) + pow(y - point.second, 2));

  EXPECT_NEAR(collectionRange, distance, 0.1);
}
