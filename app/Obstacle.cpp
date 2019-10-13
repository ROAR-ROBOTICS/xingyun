/**Copyright (c) 2019 Hao Da (Kevin) Dong, Zuyang Cao, Jing Liang
 * @file       Obstacle.cpp
 * @date       10/13/2019
 * @brief      The file Obstacle.cpp implements the obstacle class.
 *             The class will be used in Xingyun class.
 * @license    This project is released under the BSD-3-Clause License.
 */
#pragma once

#include<iostream>
#include<vector>

class Obstacle{
 public:
  /** @brief The left most point in the obstacle cluster. */
  std::vector<double> leftMostPoint;

  /** @brief The right most point in the obstacle cluster. */
  std::vector<double> rightMostPoint;

  /** @brief The midpoint in the obstacle cluster. */
  std::vector<double> midpoint;

  /** @brief The largest gradient in the obstacle cluster. */
  double largestGrad;

  /** @brief The smallest gradient in the obstacle cluster. */
  double smallestGrad;
};
