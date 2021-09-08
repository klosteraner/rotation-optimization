#include "setupScene.h"
#include "optimization.h"
#include "types.h"

#include "sophus/common.hpp"

#include <iostream>
#include <vector>

#include <Eigen/Geometry>

int main()
{
	//using Rotation = sote::AngleAxisRotation; // Convenience switch
	using Rotation = sote::QuaternionRotation; // Convenience switch

	sote::MeasuredScene<Rotation> measurements;
	sote::OptimizedScene<Rotation> parameters;
	std::tie(measurements, parameters) = sote::setupTestScene<Rotation>();

	sote::optimize(measurements, parameters);

	return 0;
}
