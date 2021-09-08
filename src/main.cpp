#include "setupScene.h"
#include "optimization.h"
#include "types.h"

#include "sophus/common.hpp"

#include <iostream>
#include <vector>

int main()
{
	sote::MeasuredScene measurements;
	sote::OptimizedScene parameters;
	std::tie(measurements, parameters) = sote::setupTestScene();

	sote::optimize(measurements, parameters);

	return 0;
}
