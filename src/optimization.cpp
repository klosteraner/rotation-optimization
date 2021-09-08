#include "optimization.h"

#include "geometry.hpp"
#include "reprojectionError.hpp"

#include <ceres/ceres.h>

#include <chrono>

using namespace std::chrono;

namespace sote
{
  namespace
  {
    void addAngleAxisReprojectionError(ceres::Problem& problem, const MeasuredScene& measurements, OptimizedScene& parameters)
    {
      for(const auto& track : measurements.tracks)
      {
        for(const auto& camIdAndMark : track.marks)
        {
          const auto& camera = measurements.cameras.at(camIdAndMark.first);

          ceres::CostFunction* f =
            new ceres::AutoDiffCostFunction<AngleAxisReprojectionCostFunctorAuto, 2, 3>(
              new AngleAxisReprojectionCostFunctorAuto(camera.pose.position,
                                                       measurements.cameraSensors.at(camera.sensorId),
                                                       camIdAndMark.second,
                                                       track.originalPosition));

          problem.AddResidualBlock(f, nullptr, parameters.rotation[camIdAndMark.first].data());
        }
      }
    }

    void setupAngleAxisProblem(ceres::Problem& problem, const MeasuredScene& measurements, OptimizedScene& parameters)
    {
      addAngleAxisReprojectionError(problem, measurements, parameters);
    }

    void setupProblem(ceres::Problem& problem, const MeasuredScene& measurements, OptimizedScene& parameters)
    {
      setupAngleAxisProblem(problem, measurements, parameters);
    }
  }

  void optimize(const MeasuredScene& measurements, OptimizedScene& parameters)
  {

  	ceres::Problem problem;
    setupProblem(problem, measurements, parameters);

    /*
     *  2. Solve problem
     */
  	ceres::Solver::Options options;
  	options.minimizer_progress_to_stdout = true;
    options.function_tolerance = 1e-13;
  	ceres::Solver::Summary summary;

    steady_clock::time_point t1 = steady_clock::now();
  	ceres::Solve(options, &problem, &summary);
    steady_clock::time_point t2 = steady_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "It took me " << time_span.count() << " seconds." << std::endl;

  	std::cout << summary.FullReport() << std::endl;
    for(int i = 0; i < measurements.cameras.size(); i++)
    {
      std::cout << "IMU: " << measurements.cameras.at(i).pose.rotation(0) << " Opt: " << parameters.rotation.at(i)(0) << std::endl;
      std::cout << "IMU: " << measurements.cameras.at(i).pose.rotation(1) << " Opt: " << parameters.rotation.at(i)(1) << std::endl;
      std::cout << "IMU: " << measurements.cameras.at(i).pose.rotation(2) << " Opt: " << parameters.rotation.at(i)(2) << std::endl;
      std::cout << "-----------------------------------------" << std::endl;
    }
  }
}
