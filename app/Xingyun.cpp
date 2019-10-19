/**Copyright (c) 2019 Hao Da (Kevin) Dong, Zuyang Cao, Jing Liang
 * @file       Xingyun.cpp
 * @date       10/13/2019
 * @brief      The file Xingyun.cpp implements a human perception class.
 *             The class will be used for detecting human from 2D Lidar data-sets.
 * @license    This project is released under the BSD-3-Clause License.
 */
#include <iostream>
#include <vector>
#include <iterator>
#include <fstream>
#include <math.h>
#include <algorithm>
#include <string>
#include <Obstacle.hpp>
#include <Human.hpp>
#include <Xingyun.hpp>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

/** @brief Read data and classify the points into obstacles. */
void Xingyun::obstacleClassification() {
  return;
}

/** @brief Recognize legs among obstacles. */
void Xingyun::legRecognition() {
  return;
}

/** @brief Recognize humans from legs. */
void Xingyun::humanRecognition() {
  return;
}

/**
 * @brief Main function to detect human.
 * @param lidarDatasetFilename - Name of the data-set.
 * @return humanList - The detected human list.
 */
std::vector<Human> Xingyun::humanPerception(std::string lidarDatasetFilename) {
  Human person;
  std::ifstream is(lidarDatasetFilename);
  std::istream_iterator<double> start(is), end;
  std::vector<double> tmpList(start, end);
  rawLidarDistances = tmpList;  // Load raw data in vector;
  std::vector<double> degList;
  double degDelt = 240.0 / 512.0;
  for (int i = 0; i < 512; i++) {
    degList.emplace_back(-120.0 + degDelt * i);
  }
  pointCloudPolar.emplace_back(degList);
  pointCloudPolar.emplace_back(rawLidarDistances);
  std::vector<double> xList;
  std::vector<double> yList;
  for (int i = 0; i < 512; i++) {
    double cartX = pointCloudPolar[1][i]
        * cos(pointCloudPolar[0][i] * M_PI / 180.0);
    double cartY = pointCloudPolar[1][i]
        * sin(pointCloudPolar[0][i] * M_PI / 180.0);
    xList.emplace_back(cartX);
    yList.emplace_back(cartY);
  }
  pointCloudCartesian.emplace_back(xList);
  pointCloudCartesian.emplace_back(yList);
  // Start classification and recognition.
  obstacleClassification();
  legRecognition();
  humanRecognition();
  return humanList;
}

/** @brief Show the output map. */
void Xingyun::visualization() {
  plt::plot( { 0 }, { 0 }, "bs");  // Show robot as square at origin.
  double humanWidth = 0.8;  // Human width, adjust here.
  double humanThick = 0.3;  // Human thickness, adjust here.
  for (auto human : humanList) {
    plt::plot( { human.centroid[0] }, { human.centroid[1] }, "ro");  // Plot human centroid in map.
    double orientation = human.orientationAngle;  // Human orientation in degree, x axis(pointing to the right) is 0 degree.
    int n = 500;
    std::vector<double> x(n), y(n);
    for (int i = 0; i < n; ++i) {
      double t = 2 * M_PI * i / n;
      x.at(i) = humanThick * cos(t) * cos(orientation / 180.0 * M_PI)
              - humanWidth * sin(t) * sin(orientation / 180.0 * M_PI) + human.centroid[0];
      y.at(i) = humanThick * cos(t) * sin(orientation / 180.0 * M_PI)
              + humanWidth * sin(t) * cos(orientation / 180.0 * M_PI) + human.centroid[1];
    }
    plt::plot(x, y, "r-");  // Plot human as ellipse.
  }
  plt::xlim(-4, 4);
  plt::ylim(-4, 4);
  plt::show();
  return;
}

/** @brief Get rawLidarDistances.
 *  @return rawLidarDistances - Raw data.
 */
std::vector<double> Xingyun::getRawLidarDistances() {
  return rawLidarDistances;
}

/** @brief Get pointCloudPolar.
 *  @return pointCloudPolar - Converted data in Polar coordinate.
 */
std::vector<std::vector<double>> Xingyun::getPointCloudPolar() {
  return pointCloudPolar;
}

/** @brief Get pointCloudCartesian.
 *  @return pointCloudCartesian - Converted data in Cartesian coordinate.
 */
std::vector<std::vector<double>> Xingyun::getPointCloudCartesian() {
  return pointCloudCartesian;
}

/** @brief Get obstacleList.
 *  @return obstacleList - List of obstacles.
 */
std::vector<Obstacle> Xingyun::getObstacleList() {
  return obstacleList;
}

/** @brief Get legList.
 *  @return legList - List of legs.
 */
std::vector<Obstacle> Xingyun::getLegList() {
  return legList;
}

/** @brief Get humanList.
 *  @return humanList - List of humans.
 */
std::vector<Human> Xingyun::getHumanList() {
  return humanList;
}

