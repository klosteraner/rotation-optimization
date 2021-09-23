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
    void addAngleAxisReprojectionError(ceres::Problem& problem, const MeasuredScene<AngleAxisRotation>& measurements, OptimizedScene<AngleAxisRotation>& parameters)
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

    void addQuaternionReprojectionError(ceres::Problem& problem, const MeasuredScene<QuaternionRotation>& measurements, OptimizedScene<QuaternionRotation>& parameters, bool useAnalyticDerivative = true)
    {
      for(const auto& track : measurements.tracks)
      {
        for(const auto& camIdAndMark : track.marks)
        {
          const auto& camera = measurements.cameras.at(camIdAndMark.first);

          ceres::CostFunction* f;
          if(useAnalyticDerivative)
          {
            f = new QuaternionReprojectionCostFunctionAna(camera.pose.position,
                                                          measurements.cameraSensors.at(camera.sensorId),
                                                          camIdAndMark.second,
                                                          track.originalPosition);
          }
          else
          {
            f = new ceres::AutoDiffCostFunction<QuaternionReprojectionCostFunctorAuto, 2, 4>(
                  new QuaternionReprojectionCostFunctorAuto(camera.pose.position,
                                                            measurements.cameraSensors.at(camera.sensorId),
                                                            camIdAndMark.second,
                                                            track.originalPosition));
          }

          problem.AddParameterBlock(parameters.rotation[camIdAndMark.first].coeffs().data(), 4, new ceres::EigenQuaternionParameterization());
          problem.AddResidualBlock(f, nullptr, parameters.rotation[camIdAndMark.first].coeffs().data());
        }
      }
    }

    template<typename RotationType> void setupProblem(ceres::Problem& problem, const MeasuredScene<RotationType>& measurements, OptimizedScene<RotationType>& parameters);
    template<> void setupProblem(ceres::Problem& problem, const MeasuredScene<AngleAxisRotation>& measurements, OptimizedScene<AngleAxisRotation>& parameters)
    {
      addAngleAxisReprojectionError(problem, measurements, parameters);
    }
    template<> void setupProblem(ceres::Problem& problem, const MeasuredScene<QuaternionRotation>& measurements, OptimizedScene<QuaternionRotation>& parameters)
    {
      addQuaternionReprojectionError(problem, measurements, parameters);
    }

    template<typename RotationType> void print(const RotationType& expectedRotation, const RotationType& initialRotation, const RotationType& optimizedRotation);
    template<> void print(const AngleAxisRotation& expectedRotation, const AngleAxisRotation& initialRotation, const AngleAxisRotation& optimizedRotation)
    {
      std::cout << "Initial: " << initialRotation(0) << " -> Expected: " << expectedRotation(0) << " Optimized: " << optimizedRotation(0) << std::endl;
      std::cout << "Initial: " << initialRotation(1) << " -> Expected: " << expectedRotation(1) << " Optimized: " << optimizedRotation(1) << std::endl;
      std::cout << "Initial: " << initialRotation(2) << " -> Expected: " << expectedRotation(2) << " Optimized: " << optimizedRotation(2) << std::endl;
      std::cout << "-----------------------------------------" << std::endl;
    }
    template<> void print(const QuaternionRotation& expectedRotation, const QuaternionRotation& initialRotation, const QuaternionRotation& optimizedRotation)
    {
      std::cout << "Initial: " << initialRotation.x() << " -> Expected: " << expectedRotation.x() << " Optimized: " << optimizedRotation.x() << std::endl;
      std::cout << "Initial: " << initialRotation.y() << " -> Expected: " << expectedRotation.y() << " Optimized: " << optimizedRotation.y() << std::endl;
      std::cout << "Initial: " << initialRotation.z() << " -> Expected: " << expectedRotation.z() << " Optimized: " << optimizedRotation.z() << std::endl;
      std::cout << "Initial: " << initialRotation.w() << " -> Expected: " << expectedRotation.w() << " Optimized: " << optimizedRotation.w() << std::endl;
      std::cout << "-----------------------------------------" << std::endl;
    }
  }

  template<typename RotationType>
  void optimizeImpl(const MeasuredScene<RotationType>& measurements, OptimizedScene<RotationType>& parameters)
  {
    auto initialParameters = parameters;

  	ceres::Problem problem;
    setupProblem(problem, measurements, parameters);

    /*
     *  2. Solve problem
     */
  	ceres::Solver::Options options;
  	options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100;
    options.function_tolerance = 1e-15;
  	ceres::Solver::Summary summary;

    steady_clock::time_point t1 = steady_clock::now();
  	ceres::Solve(options, &problem, &summary);
    steady_clock::time_point t2 = steady_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "It took me " << time_span.count() << " seconds." << std::endl;

  	std::cout << summary.FullReport() << std::endl;
    for(int i = 0; i < measurements.cameras.size(); i++)
    {
      print(measurements.cameras.at(i).pose.rotation, initialParameters.rotation.at(i), parameters.rotation.at(i));
    }
  }

  template<> void optimize(const MeasuredScene<AngleAxisRotation>& measurements, OptimizedScene<AngleAxisRotation>& parameters)
  {
    optimizeImpl(measurements, parameters);
  }
  template<> void optimize(const MeasuredScene<QuaternionRotation>& measurements, OptimizedScene<QuaternionRotation>& parameters)
  {
    optimizeImpl(measurements, parameters);
  }
}
