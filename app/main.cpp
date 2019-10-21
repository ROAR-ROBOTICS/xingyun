#include <gtest/gtest.h>
#include <Xingyun.hpp>
#include <Obstacle.hpp>
#include <Human.hpp>

#include "Xingyun.cpp"

#include <vector>
#include <fstream>
#include <sstream>
#include <iterator>
#include <string>
#include <boost/range/combine.hpp>
#include <boost/tuple/tuple.hpp>

int main() {
	Xingyun xingyun;
	std::string fileName = "dataset/test_single.csv";
	std::vector<Human> humanInfo = xingyun.humanPerception(fileName);

  return 0;
}
