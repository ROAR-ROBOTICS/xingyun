/**Copyright (c) 2019 Hao Da (Kevin) Dong, Zuyang Cao, Jing Liang
 * @file       Xingyun.cpp
 * @date       10/13/2019
 * @brief      The file Xingyun.cpp implements a human perception class.
 *             The class will be used for detecting human from 2D Lidar data-sets.
 * @license    This project is released under the BSD-3-Clause License.
 */
#pragma once

#include<iostream>
#include<vector>
#include<string>
#include"Obstacle.hpp"
#include"Human.hpp"

class Xingyun{
 private:
  /** @brief Raw Lidar distances data read from data file. */
  std::vector<double> rawLidarDistances;

  /** @brief Point cloud data converted into polar coordinate. */
  std::vector<std::vector<double>> pointCloudPolar;

  /** @brief Point cloud data converted into Cartesian coordinate. */
  std::vector<std::vector<double>> pointCloudCartesian;

  /** @brief List to store obstacles. */
  std::vector<Obstacle> obstacleList;

  /** @brief List to store detected legs. */
  std::vector<Obstacle> legList;

  /** @brief List to store detected humans. */
  std::vector<Human> humanList;

  /** @brief Read data and classify the points into obstacles. */
  void obstacleClassification(){
    return;
  }

  /** @brief Recognize legs among obstacles. */
  void legRecognition(){
    return;
  }

  /** @brief Recognize humans from legs. */
  void humanRecognition(){
    return;
  }

 public:
  /**
   * @brief Main function to detect human.
   * @param lidarDatasetFilename - Name of the data-set.
   * @return humanList - The detected human list.
   */
  std::vector<Human> humanPerception(std::string lidarDatasetFilename){
    return humanList;
  }

  /** @brief Show the output map. */
  void visualization(){
    return;
  }

  /** @brief Get rawLidarDistances.
   *  @return rawLidarDistances - Raw data.
   */
  std::vector<double> getRawLidarDistances(){
    return rawLidarDistances;
  }

  /** @brief Get pointCloudPolar.
   *  @return pointCloudPolar - Converted data in Polar coordinate.
   */
  std::vector<std::vector<double>> getPointCloudPolar(){
    return pointCloudPolar;
  }

  /** @brief Get pointCloudCartesian.
   *  @return pointCloudCartesian - Converted data in Cartesian coordinate.
   */
  std::vector<std::vector<double>> getPointCloudCartesian(){
    return pointCloudCartesian;
  }

  /** @brief Get obstacleList.
   *  @return obstacleList - List of obstacles.
   */
  std::vector<Obstacle> getObstacleList(){
    return obstacleList;
  }

  /** @brief Get legList.
   *  @return legList - List of legs.
   */
  std::vector<Obstacle> getLegList(){
    return legList;
  }

  /** @brief Get humanList.
   *  @return humanList - List of humans.
   */
  std::vector<Human> getHumanList(){
    return humanList;
  }
};
