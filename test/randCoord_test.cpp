/**
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file randCoord_test.cpp
 * @brief Test of randCoord class
 * @author Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 */

#include "randCoord.hpp"
#include <gtest/gtest.h>
#include <vector>

/**
 * @brief Test Constructor of randCoord class
 *
 * @param[in]     RandCoordTest
 * @param[in]     testConstructor
 *
 * @return     none
 */
TEST(RandCoordTest, testConstructor) {
  /// Center of map vector
  std::vector<float> centerPt = {0, 0};

  /// randCoord object
  randCoord random(10.0, 10.0, centerPt);

  /// Check if constructor sets members
  EXPECT_EQ(10.0, random.width_);
  EXPECT_EQ(10.0, random.length_);
  EXPECT_EQ(centerPt.size(), random.centerPoint.size());
}

/**
 * @brief Test randX() method of randCoord class
 *
 * @param[in]     DetectorTest
 * @param[in]     testRandX
 *
 * @return     none
 */
TEST(DetectorTest, testRandX) {
  /// Center of map vector
  std::vector<float> centerPt = {0, 0};

  /// randCoord object
  randCoord random(10.0, 8.0, centerPt);

  /// Creating random x coords
  float x = random.randX();
  float x1 = random.randX();
  float x2 = random.randX();

  /// Check if random x coord is within map range
  EXPECT_TRUE((x >= -5.0) && (x <= 5.0));
  EXPECT_TRUE((x1 >= -5.0) && (x1 <= 5.0));
  EXPECT_TRUE((x2 >= -5.0) && (x2 <= 5.0));
}

/**
 * @brief Test randy() method of randCoord class
 *
 * @param[in]     DetectorTest
 * @param[in]     testRandY
 *
 * @return     none
 */
TEST(DetectorTest, testRandY) {
  /// Center of map vector
  std::vector<float> centerPt = {0, 0};

  /// randCoord object
  randCoord random(10.0, 8.0, centerPt);

  /// Creating random y coords
  float y = random.randY();
  float y1 = random.randY();
  float y2 = random.randY();

  /// Check if random y coord is within map range
  EXPECT_TRUE((y >= -4.0) && (y <= 4.0));
  EXPECT_TRUE((y1 >= -4.0) && (y1 <= 4.0));
  EXPECT_TRUE((y2 >= -4.0) && (y2 <= 4.0));
}
