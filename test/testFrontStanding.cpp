/**Copyright (c) 2019 Hao Da (Kevin) Dong, Zuyang Cao, Jing Liang
 * @file       Xingyun.hpp
 * @date       10/13/2019
 * @brief      This file is to test functions of the project in scenario where 
 *             person stand in front of the lidar. The file test if lidar can
 *             recognize people correct
 * @license    This project is released under the BSD-3-Clause License.
 */
#include <gtest/gtest.h>
#include <Xingyun.hpp>
#include <Obstacle.hpp>
#include <Human.hpp>

/**
 * @brief test value in x axis of detected human position
 * @param testPointCloudPolar
 * @param should_pass
 * @return computes error
 */
TEST(testFrontHumanPositionX, should_pass) {
  Xingyun xingyun;
  std::string filename = "dataset/test_double.csv";
  std::vector<Human> humanInfo = xingyun.humanPerception(filename);
  std::vector<double> midpoint = humanInfo[0].centroid;

  EXPECT_NEAR(midpoint[0], 2,0.2);
}

/**
 * @brief test value in y axis of detected human position
 * @param testPointCloudPolar
 * @param should_pass
 * @return computes error
 */
TEST(testFrontHumanPositionY, should_pass) {
  Xingyun xingyun;
  std::string filename = "dataset/test_double.csv";
  std::vector<Human> humanInfo = xingyun.humanPerception(filename);
  std::vector<double> midpoint = humanInfo[0].centroid;

  EXPECT_NEAR(midpoint[1], 0,0.2);
}


/**
 * @brief test value in yaw axis of detected human 
 * @param testPointCloudPolar
 * @param should_pass
 * @return computes error
 */
TEST(testFrontHumanPoseYaw, should_pass) {
  Xingyun xingyun;
  std::string filename = "dataset/test_double.csv";
  std::vector<Human> humanInfo = xingyun.humanPerception(filename);
  double orientationAngle = humanInfo[0].orientationAngle;

  EXPECT_NEAR(orientationAngle, 0,0.2);
}