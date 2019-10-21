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
#include <sstream>
#include <math.h>
#include <algorithm>
#include <string>
#include <boost/range/combine.hpp>
#include <boost/tuple/tuple.hpp>

#include <Obstacle.hpp>
#include <Human.hpp>
#include <Xingyun.hpp>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

#define LIDAR_RANGE 3.9
#define CLUSTER_THRESHOLD 0.5
#define LEG_DIAMETER 0.2
#define LEG_DISTANCE 0.3
#define MAJOR_AXIS 0.6
#define MINOR_AXIS 0.4
#define GRAD_DIFF_THRESHOLD 0.1

/**
 * @brief The function is used to generate angles in polar coordinates
 * @param none
 * @return angle - The calculated angle for the point.
 */
double calculateAngle() {
	static double i=0;
	double angle = (-120+i*240/512)*M_PI/180;
	i++;
	return angle;
}


/** @brief Read data and classify the points into obstacles. */
void Xingyun::obstacleClassification() {

	double preDistance = rawLidarDistances[0];
	std::vector<double> xList,yList,distanceList;
	std::vector<std::vector<double>> objectBufferX,objectBufferY;
	std::vector<std::vector<std::vector<double>>> objects;
	
	// point cloud classification
	for(auto const& tupleValue: boost::combine(rawLidarDistances,pointCloudCartesian[0],pointCloudCartesian[1])) {
		double distance, x, y;
		boost::tie(distance,x,y) = tupleValue;

		if (abs(distance - preDistance)>= CLUSTER_THRESHOLD) {
			if (distanceList.size()>0) {
				if (distanceList[0] < LIDAR_RANGE){
					objectBufferX.push_back(xList);
					objectBufferY.push_back(yList);
				}
				xList.clear();
				yList.clear();
				distanceList.clear();
			}
		}
		preDistance = distance;
		xList.push_back(x);
		yList.push_back(y);
		distanceList.push_back(distance);
	}
	if (distanceList.size()>0) {
		if (distanceList[0] < LIDAR_RANGE){
			objectBufferX.push_back(xList);
			objectBufferY.push_back(yList);
		}
	}

	// extract information needed for obstacle object
	for(auto const& tupleValue: boost::combine(objectBufferX, objectBufferY)) {
		std::vector<double> xTempList,yTempList,cartesianVector;
		Obstacle obstacleValue;

		boost::tie(xTempList,yTempList) = tupleValue;
		obstacleValue.rightMostPoint = {*(xTempList.end()-1),*(yTempList.end()-1)};
		obstacleValue.leftMostPoint =  {*xTempList.begin(),*yTempList.begin()};
		obstacleValue.midPoint = {(*xTempList.begin()+*(xTempList.end()-1))/2, (*(yTempList.end()-1)+*yTempList.begin())/2 };

		std::vector<double> gradiences;
		auto const tupleCombination = boost::combine(xTempList, yTempList);
		for(auto beginIndex = tupleCombination.begin();beginIndex<tupleCombination.end()-1;beginIndex++) {
			double x,y,nextX,nextY;
			boost::tie(x,y) = *beginIndex;
			boost::tie(nextX,nextY) = *(beginIndex+1);

			double gradience;
			if ((nextX-x)==0) gradience = 9999;
			else gradience = (nextY-y)/(nextX-x);
			if (gradience>9999) gradience = 9999;
			gradiences.push_back(gradience);
		}
		obstacleValue.largestGrad = *std::max_element(gradiences.begin(),gradiences.end());
		obstacleValue.smallestGrad = *std::min_element(gradiences.begin(),gradiences.end());

		obstacleList.push_back(obstacleValue);
	}

	return;
}


/** @brief Recognize legs among obstacles. */
void Xingyun::legRecognition() {
    double obstacleLength = 0;   // Length of obstacle contour

    // Check each obstacle in obstacleList
    for (auto const obstacle : obstacleList) {
        // Check gradient constraint
        if (obstacle.largestGrad - obstacle.smallestGrad > GRAD_DIFF_THRESHOLD) {
            // Calculate length of obstacle contour
            obstacleLength = sqrt(pow(obstacle.rightMostPoint[0] - obstacle.leftMostPoint[0], 2)
                                    + pow(obstacle.rightMostPoint[1] - obstacle.leftMostPoint[1], 2));

            // Check diameter constraint. Add obstacle to legList if it passes both constraints.
            if (obstacleLength < LEG_DIAMETER)
                legList.push_back(obstacle);
        }
    }
}


/** @brief Supporting function to humanRecognition() to process humans where both legs are visible
 *  @param queue 2-element inspection queue for leg pairs
*/
void processNormalHuman(std::vector<Obstacle> queue) {
    return;
}


/** @brief Supporting function to humanRecognition() to process humans where only one leg is visible
 *  @param queue 2-element inspection queue for leg pairs
*/
void processSidewaysHuman(std::vector<Obstacle> queue) {
    return;
}


/** @brief Recognize humans from legs. */
void Xingyun::humanRecognition() {
    std::vector<Obstacle> queue;    // 2-element inspection queue for leg pairs

    while (legList.empty() == false) {

        // Pop first element of legList into the inspeaction queue
        queue.push_back(legList.front());
        legList.pop_front();

        // Load up queue to two elements, unless legList is empty
        if (queue.size() != 2) {
            if (legList.empty() == false)
                continue;
        }


    }

    
}


/**
 * @brief Main function to detect human.
 * @param lidarDatasetFilename - Name of the data-set.
 * @return humanList - The detected human list.
 */
std::vector<Human> Xingyun::humanPerception(std::string lidarDatasetFilename) {

	// load lidar data from csv file to vector rawLidarDistances
	std::ifstream fileStream(lidarDatasetFilename);
	std::string item;
	while(std::getline(fileStream, item,',')) {
		double value;
		std::stringstream readString;
		readString << item;
		readString >> value;
		rawLidarDistances.push_back(value);
	}

	//get polar data from raw lidar data
	std::vector<double> angles(512) ; // vector with 100 ints.
	std::generate(angles.begin(), angles.end(), calculateAngle);
	pointCloudPolar.push_back(rawLidarDistances);
	pointCloudPolar.push_back(angles);

	//converte polar coordinates to cartesian coordinates
	std::vector<double> xValues,yValues;
	for(auto const& tupleValue: boost::combine(rawLidarDistances, angles)) {
		double distance,angle;
		boost::tie(distance,angle) = tupleValue;
		xValues.push_back(distance * cos(angle));
		yValues.push_back(distance * sin(angle));
	}
	pointCloudCartesian.push_back(xValues);
	pointCloudCartesian.push_back(yValues);

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

