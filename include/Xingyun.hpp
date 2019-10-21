/**Copyright (c) 2019 Hao Da (Kevin) Dong, Zuyang Cao, Jing Liang
 * @file       Xingyun.hpp
 * @date       10/13/2019
 * @brief      The file Xingyun.hpp contains the header declarations for a
 *             human perception class. The class will be used for detecting
 *             human from 2D Lidar data-sets.
 * @license    This project is released under the BSD-3-Clause License.
 */
#pragma once

#include <iostream>
#include <vector>
#include <iterator>
#include <fstream>
#include <sstream>
#include <math.h>
#include <algorithm>
#include <string>
#include <boost/range/combine.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/range/irange.hpp>

#include <Obstacle.hpp>
#include <Human.hpp>

//#include <matplotlibcpp.h>
//
//namespace plt = matplotlibcpp;

#define LIDAR_RANGE 3.9
#define CLUSTER_THRESHOLD 0.5
#define LEG_DIAMETER 0.2
#define LEG_DISTANCE 0.3
#define MAJOR_AXIS 0.6
#define MINOR_AXIS 0.4
#define GRAD_DIFF_THRESHOLD 0.1


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
        void obstacleClassification();

        /** @brief Recognize legs among obstacles. */
        void legRecognition();

        /** @brief Recognize humans from legs. */
        void humanRecognition();

        /** @brief Supporting function to humanRecognition() to process humans where both legs are visible
         *  @param queue 2-element inspection queue for leg pairs
        */
        void processNormalHuman(std::vector<Obstacle> queue);

        /** @brief Supporting function to humanRecognition() to process humans where only one leg is visible
         *  @param queue 2-element inspection queue for leg pairs
        */
        void processSidewaysHuman(std::vector<Obstacle> queue);

    public:
        /**
         * @brief Main function to detect human.
         * @param lidarDatasetFilename Name of the data-set.
         * @return humanList - The detected human list.
         */
        std::vector<Human> humanPerception(std::string lidarDatasetFilename);

        /** @brief Show the output map. */
        void visualization();

        /** @brief Get rawLidarDistances.
         *  @return rawLidarDistances - Raw data.
         */
        std::vector<double> getRawLidarDistances();

        /** @brief Get pointCloudPolar.
         *  @return pointCloudPolar - Converted data in Polar coordinate.
         */
        std::vector<std::vector<double>> getPointCloudPolar();

        /** @brief Get pointCloudCartesian.
         *  @return pointCloudCartesian - Converted data in Cartesian coordinate.
         */
        std::vector<std::vector<double>> getPointCloudCartesian();

        /** @brief Get obstacleList.
         *  @return obstacleList - List of obstacles.
         */
        std::vector<Obstacle> getObstacleList();

        /** @brief Get legList.
         *  @return legList - List of legs.
         */
        std::vector<Obstacle> getLegList();

        /** @brief Get humanList.
         *  @return humanList - List of humans.
         */
        std::vector<Human> getHumanList();
};
