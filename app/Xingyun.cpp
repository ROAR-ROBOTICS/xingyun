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
#include <Obstacle.hpp>
#include <Human.hpp>
#include <Xingyun.hpp>


  /** @brief Read data and classify the points into obstacles. */
  void Xingyun::obstacleClassification(){
    return;
  }

  /** @brief Recognize legs among obstacles. */
  void Xingyun::legRecognition(){
    return;
  }

  /** @brief Recognize humans from legs. */
  void Xingyun::humanRecognition(){
    return;
  }

  /**
   * @brief Main function to detect human.
   * @param lidarDatasetFilename - Name of the data-set.
   * @return humanList - The detected human list.
   */
  std::vector<Human> Xingyun::humanPerception(std::string lidarDatasetFilename){
    Human person;
    person.centroid.push_back(2.0);
    person.centroid.push_back(0.0);
    humanList.push_back(person);
    return humanList;
  }

  /** @brief Show the output map. */
  void Xingyun::visualization(){
    return;
  }

  /** @brief Get rawLidarDistances.
   *  @return rawLidarDistances - Raw data.
   */
  std::vector<double> Xingyun::getRawLidarDistances(){
    return rawLidarDistances;
  }

  /** @brief Get pointCloudPolar.
   *  @return pointCloudPolar - Converted data in Polar coordinate.
   */
  std::vector<std::vector<double>> Xingyun::getPointCloudPolar(){
    return pointCloudPolar;
  }

  /** @brief Get pointCloudCartesian.
   *  @return pointCloudCartesian - Converted data in Cartesian coordinate.
   */
  std::vector<std::vector<double>> Xingyun::getPointCloudCartesian(){
    return pointCloudCartesian;
  }

  /** @brief Get obstacleList.
   *  @return obstacleList - List of obstacles.
   */
  std::vector<Obstacle> Xingyun::getObstacleList(){
    return obstacleList;
  }

  /** @brief Get legList.
   *  @return legList - List of legs.
   */
  std::vector<Obstacle> Xingyun::getLegList(){
    return legList;
  }

  /** @brief Get humanList.
   *  @return humanList - List of humans.
   */
  std::vector<Human> Xingyun::getHumanList(){
    return humanList;
  }

