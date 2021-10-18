#include "setupScene.h"
#include "optimization.h"
#include "types.h"

#include <iostream>
#include <vector>

#include <Eigen/Geometry>

int main()
{
	//using Rotation = roto::AngleAxisRotation; // Convenience switch
	using Rotation = roto::QuaternionRotation; // Convenience switch

	roto::MeasuredScene<Rotation> measurements;
	roto::OptimizedScene<Rotation> parameters;

	//std::tie(measurements, parameters) = roto::setupSmallTestScene<Rotation>();
	std::tie(measurements, parameters) = roto::setupBigTestScene<Rotation>();

	roto::optimize(measurements, parameters);

	return 0;
}
