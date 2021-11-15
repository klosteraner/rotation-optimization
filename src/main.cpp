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
	//using Rotation = roto::MatrixRotation; // Convenience switch

	roto::MeasuredScene<Rotation> measurements;
	roto::OptimizedScene<Rotation> parameters;

	double totalTime = 0.;
	const std::size_t numberOfIterations = 100;
	for(std::size_t i = 0; i < numberOfIterations; i++)
	{
		//std::tie(measurements, parameters) = roto::setupSmallTestScene<Rotation>();
		std::tie(measurements, parameters) = roto::setupBigTestScene<Rotation>();

		totalTime += roto::optimize(measurements, parameters);
	}
	std::cout << "Average processing time: "
						<< totalTime / double(numberOfIterations) << std::endl;

	return 0;
}
