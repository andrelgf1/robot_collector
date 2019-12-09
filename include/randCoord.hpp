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

#ifndef INCLUDE_RANDCOORD_HPP_
#define INCLUDE_RANDCOORD_HPP_

#include <math.h>
#include <random>
#include <vector>

/**
 * @brief Class to create random (x,y) coordinates
 */
class randCoord {
 public:
  /// Private characteristics of the map to generate coordinates accordingly
  float width_;
  float length_;
  std::vector<float> centerPoint;

  /**
   * @brief Constructor for randCoord
   */
  randCoord(float width, float length, std::vector<float> midPoint);

  /**
   * @brief Method that returns a random x coordinate
   * @param none
   * @return float returns a float x coordinate
   */
  float randX();

  /**
   * @brief Method that returns a random y coordinate
   * @param none
   * @return float returns a float y coordinate
   */
  float randY();
};

#endif  // INCLUDE_RANDCOORD_HPP_
