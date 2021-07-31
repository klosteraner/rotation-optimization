#include <iostream>

#include "sophus/common.hpp"

#include <ceres/ceres.h>

int main()
{
	
	ceres::Problem problem;

	const double kPi = Sophus::Constants<double>::pi();
	std::cout << "hello pi" << kPi << std::endl;
	return 0;
}
