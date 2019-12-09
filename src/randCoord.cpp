/**
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file objectHandling.cpp
 * @brief Source code for object handling class
 * @author Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 */

#include "randCoord.hpp"

/**
 * @brief Constructor for randCoord
 */
randCoord::randCoord(float width, float length, std::vector<float> midPoint):
  width_(width), length_(length), centerPoint(midPoint) {
}


/**
 * @brief Method that returns a random x coordinate
 */
float randCoord::randX() {
  /// Random seed to generate coordinate
  std::random_device rd;
  std::mt19937 mt(rd());

  /// Range of random numbers that could be generated
  std::uniform_real_distribution<float> distX((centerPoint[0] - width_/2.0),
    (centerPoint[0] + width_/2.0));

  return distX(mt);
}

/**
 * @brief Method that returns a random y coordinate
 */
float randCoord::randY() {
  /// Random seed to generate coordinate
  std::random_device rd;
  std::mt19937 mt(rd());

  /// Range of random numbers that could be generated
  std::uniform_real_distribution<float> distY((centerPoint[1] - length_/2.0),
    (centerPoint[1] + length_/2.0));

  return distY(mt);
}
