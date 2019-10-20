/**Copyright (c) 2019 Hao Da (Kevin) Dong, Zuyang Cao, Jing Liang
 * @file       Xingyun.hpp
 * @date       10/13/2019
 * @brief      This file is to test functions of the project in the scenario  
 *             where person stand a side the lidar. The file test if lidar can
 *             recognize people correct when lidar can only detect one leg.
 * @license    This project is released under the BSD-3-Clause License.
 */
#include <gtest/gtest.h>
#include <Xingyun.hpp>
#include <Obstacle.hpp>
#include <Human.hpp>

#include <vector>
#include <fstream>
#include <sstream>
#include <iterator>
#include <string>

/**
 * @brief test value in x axis of detected human position
 * @param testPointCloudPolar
 * @param shouldPass
 * @return computes error
 */
TEST(testSideHumanPositionX, shouldPass) {
  Xingyun xingyun;
  std::string fileName = "dataset/test_single.csv";
  std::vector<Human> humanInfo = xingyun.humanPerception(fileName);
  std::vector<double> midpoint = humanInfo[0].centroid;

  EXPECT_NEAR(midpoint[0], 2,0.2);
}

/**
 * @brief test value in y axis of detected human position
 * @param testPointCloudPolar
 * @param shouldPass
 * @return computes error
 */
TEST(testSideHumanPositionY, shouldPass) {
  Xingyun xingyun;
  std::string fileName = "dataset/test_single.csv";
  std::vector<Human> humanInfo = xingyun.humanPerception(fileName);
  std::vector<double> midpoint = humanInfo[0].centroid;

  EXPECT_NEAR(midpoint[1], 0,0.2);
}


/**
 * @brief test value in yaw axis of detected human
 * @param testPointCloudPolar
 * @param shouldPass
 * @return computes error
 */
TEST(testSideHumanPoseYaw, shouldPass) {
  Xingyun xingyun;
  std::string fileName = "dataset/test_single.csv";
  std::vector<Human> humanInfo = xingyun.humanPerception(fileName);
  double orientationAngle = humanInfo[0].orientationAngle;

  EXPECT_NEAR(orientationAngle, 1.57,0.2);
} 



/**
 * @brief function to compare two Obstacle objects
 * @param Obstacle object of ground truth
 * @param Obstacle object of testing data
 * @return judgement if obstacle is same as groundtruth
 */
bool testSideObstacle(Obstacle groundTruth, Obstacle testingData) {
	bool returnValue = true;

	// test if left point is right
	if(testingData.leftMostPoint == groundTruth.leftMostPoint) {}
	else {
		returnValue = false;
		std::cout<<"left most point is not right"<<std::endl;
	}

	// test if right point is right
	if(testingData.rightMostPoint == groundTruth.rightMostPoint) {}
	else {
		returnValue = false;
		std::cout<<"right most point is not right"<<std::endl;
	}

	// test if mid point is right
	if(testingData.midPoint == groundTruth.midPoint) {}
	else {
		returnValue = false;
		std::cout<<"middle point is not right"<<std::endl;
	}

	// test if max gradience is right
	if(testingData.largestGrad == groundTruth.largestGrad) {}
	else {
		returnValue = false;
		std::cout<<"largestGrad is not right"<<std::endl;
	}

	// test if min gradience is right
	if(testingData.smallestGrad == groundTruth.smallestGrad) {}
	else {
		returnValue = false;
		std::cout<<"smallestGrad is not right"<<std::endl;
	}
	return returnValue;
}

/**
 * @brief test if tested obstacle is right
 * @param testSideObstacles
 * @param shouldPass
 * @return computes error
 */
TEST(testSideObstacles, shouldPass) {
	Xingyun xingyun;
	std::string fileName = "dataset/test_single.csv";
	std::vector<Human> humanInfo = xingyun.humanPerception(fileName);

	Obstacle groundTruth;
	groundTruth.largestGrad = 0.03;
	groundTruth.smallestGrad = 0.01;
	groundTruth.leftMostPoint.push_back(1.90);
	groundTruth.leftMostPoint.push_back(-0.17);
	groundTruth.rightMostPoint.push_back(1.92);
	groundTruth.rightMostPoint.push_back(-0.06);
	groundTruth.midPoint.push_back(1.91);
	groundTruth.midPoint.push_back(-0.12);

	std::vector<Obstacle> testingData = xingyun.getObstacleList();
	bool testCondition = testSideObstacle(groundTruth, testingData[0]);
	EXPECT_EQ(testCondition,true);
}

/**
 * @brief test if tested leg is right
 * @param testSideLegs
 * @param shouldPass
 * @return computes error
 */
TEST(testSideLegs, shouldPass) {
	Xingyun xingyun;
	std::string fileName = "dataset/test_single.csv";
	std::vector<Human> humanInfo = xingyun.humanPerception(fileName);

	Obstacle groundTruth;
	groundTruth.largestGrad = 0.03;
	groundTruth.smallestGrad = 0.01;
	groundTruth.leftMostPoint.push_back(1.90);
	groundTruth.leftMostPoint.push_back(-0.17);
	groundTruth.rightMostPoint.push_back(1.92);
	groundTruth.rightMostPoint.push_back(-0.06);
	groundTruth.midPoint.push_back(1.91);
	groundTruth.midPoint.push_back(-0.12);

	std::vector<Obstacle> testingData = xingyun.getLegList();
	bool testCondition = testSideObstacle(groundTruth, testingData[0]);
	EXPECT_EQ(testCondition,true);
}


/**
 * @brief test if polar data is right
 * @param testSidePolar
 * @param shouldPass
 * @return computes error
 */
TEST(testSidePolar, shouldPass) {

	std::ifstream file("dataset/polar_0.csv");
	if (! file) {
		std::cout << "Error, file couldn't be opened" << std::endl;
	}
	std::string readRow, item;
	std::vector<std::vector<double>> readPolar;

	while(std::getline(file, readRow)) {
		std::vector<double> readVector;
		std::stringstream ss( readRow );
		while(std::getline(ss, item,',')) {
			double value;
			std::stringstream readString;
			readString << item;
			readString >> value;
			readVector.push_back(value);
		}
		readPolar.push_back(readVector);
	}

	Xingyun xingyun;
	std::string fileName = "dataset/test_single.csv";
	std::vector<Human> humanInfo = xingyun.humanPerception(fileName);

	std::vector<std::vector<double>> testingData = xingyun.getPointCloudPolar();
	bool testCondition = (testingData == readPolar);
	EXPECT_EQ(testCondition,true);
}


/**
 * @brief test if cartesian data is right
 * @param testSideCartisian
 * @param shouldPass
 * @return computes error
 */
TEST(testSideCartisian, shouldPass) {

	std::ifstream file("dataset/cartesian_0.csv");
	if (! file) {
		std::cout << "Error, file couldn't be opened" << std::endl;
	}
	std::string readRow, item;
	std::vector<std::vector<double>> readPolar;

	while(std::getline(file, readRow)) {
		std::vector<double> readVector;
		std::stringstream ss( readRow );
		while(std::getline(ss, item,',')) {
			double value;
			std::stringstream readString;
			readString << item;
			readString >> value;
			readVector.push_back(value);
		}
		readPolar.push_back(readVector);
	}

	Xingyun xingyun;
	std::string fileName = "dataset/test_single.csv";
	std::vector<Human> humanInfo = xingyun.humanPerception(fileName);

	std::vector<std::vector<double>> testingData = xingyun.getPointCloudCartesian();
	bool testCondition = (testingData == readPolar);
	EXPECT_EQ(testCondition,true);
}
