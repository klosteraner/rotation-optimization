#include "optimization.h"

#include "geometry.hpp"
#include "reprojectionError.hpp"

#include <ceres/ceres.h>

#include <chrono>
#include <algorithm>

#include <Eigen/Geometry>

using namespace std::chrono;

namespace roto
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

      // Parametrization via exp(delta) * q (without scaling, i.e. delta = 2*angleAxis)
      problem.AddParameterBlock(parameters.rotation[camIdAndMark.first].coeffs().data(), 4, new ceres::EigenQuaternionParameterization());
      problem.AddResidualBlock(f, nullptr, parameters.rotation[camIdAndMark.first].coeffs().data());
    }
  }
}

void addMatrixReprojectionError(ceres::Problem& problem, const MeasuredScene<MatrixRotation>& measurements, OptimizedScene<MatrixRotation>& parameters, bool useAnalyticDerivative = false)
{
  for(const auto& track : measurements.tracks)
  {
    for(const auto& camIdAndMark : track.marks)
    {
      const auto& camera = measurements.cameras.at(camIdAndMark.first);

      //ceres::CostFunction* f =
      //  new ceres::AutoDiffCostFunction<AngleAxisReprojectionCostFunctorAuto, 2, 3>(
      //    new AngleAxisReprojectionCostFunctorAuto(camera.pose.position,
      //                                             measurements.cameraSensors.at(camera.sensorId),
      //                                             camIdAndMark.second,
      //                                             track.originalPosition));
      ceres::CostFunction* f;
      if(useAnalyticDerivative)
      {
        f = new RotationMatrixReprojectionCostFunctionAna2(camera.pose.position,
                                                          measurements.cameraSensors.at(camera.sensorId),
                                                          camIdAndMark.second,
                                                          track.originalPosition);
      }
      else
      {
        f = new ceres::AutoDiffCostFunction<RotationMatrixReprojectionCostFunctorAuto, 2, 9>(
              new RotationMatrixReprojectionCostFunctorAuto(camera.pose.position,
                                                            measurements.cameraSensors.at(camera.sensorId),
                                                            camIdAndMark.second,
                                                            track.originalPosition));
      }

      // Parametrization via exp(delta) * q (without scaling, i.e. delta = 2*angleAxis)
      problem.AddParameterBlock(parameters.rotation[camIdAndMark.first].data(), 9, new RotationMatrixParameterization());
      problem.AddResidualBlock(f, nullptr, parameters.rotation[camIdAndMark.first].data());
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
template<> void setupProblem(ceres::Problem& problem, const MeasuredScene<MatrixRotation>& measurements, OptimizedScene<MatrixRotation>& parameters)
{
  addMatrixReprojectionError(problem, measurements, parameters);
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
template<> void print(const MatrixRotation& expectedRotation, const MatrixRotation& initialRotation, const MatrixRotation& optimizedRotation)
{
  std::cout << "Initial: " << initialRotation.row(0) << " -> Expected: " << expectedRotation.row(0) << " Optimized: " << optimizedRotation.row(0) << std::endl;
  std::cout << "Initial: " << initialRotation.row(1) << " -> Expected: " << expectedRotation.row(1) << " Optimized: " << optimizedRotation.row(1) << std::endl;
  std::cout << "Initial: " << initialRotation.row(2) << " -> Expected: " << expectedRotation.row(2) << " Optimized: " << optimizedRotation.row(2) << std::endl;
  std::cout << "-----------------------------------------" << std::endl;
}

template<typename RotationType> void printErrorStatistics(const std::vector<Camera<RotationType> >& expectedParameters, const std::vector<RotationType>& optimizedRotations);
template<> void printErrorStatistics(const std::vector<Camera<AngleAxisRotation> >& expectedParameters, const std::vector<AngleAxisRotation>& optimizedRotations)
{
  assert(expectedParameters.size() == optimizedRotations.size());

  Eigen::Array3d meanErrorCoeffwise(0., 0., 0.);
  Eigen::Array3d maxErrorCoeffwise(0., 0., 0.);
  for(int i = 0; i < expectedParameters.size(); i++)
  {
    const Eigen::Array3d error = (expectedParameters.at(i).pose.rotation - optimizedRotations.at(i)).array();
    meanErrorCoeffwise += error;
    maxErrorCoeffwise = maxErrorCoeffwise.max(error);
  }
  meanErrorCoeffwise /= expectedParameters.size();

  std::cout << "Parameter error statistics: " << std::endl;
  std::cout << "Mean error:" << meanErrorCoeffwise << std::endl;
  std::cout << "Max error:" << maxErrorCoeffwise << std::endl;
}
template<> void printErrorStatistics(const std::vector<Camera<QuaternionRotation> >& expectedParameters, const std::vector<QuaternionRotation>& optimizedRotations)
{
  assert(expectedParameters.size() == optimizedRotations.size());

  Eigen::Array4d meanErrorCoeffwise(0., 0., 0., 0.);
  Eigen::Array4d maxErrorCoeffwise(0., 0., 0., 0.);
  for(int i = 0; i < expectedParameters.size(); i++)
  {
    const Eigen::Array4d error = (expectedParameters.at(i).pose.rotation.coeffs() - optimizedRotations.at(i).coeffs()).array().abs();
    meanErrorCoeffwise += error;
    maxErrorCoeffwise = maxErrorCoeffwise.max(error);
  }
  meanErrorCoeffwise /= expectedParameters.size();

  std::cout << "Parameter error statistics: " << std::endl;
  std::cout << "Mean error:" << std::endl << meanErrorCoeffwise << std::endl;
  std::cout << "Max error:" << std::endl << maxErrorCoeffwise << std::endl;
}
template<> void printErrorStatistics(const std::vector<Camera<MatrixRotation> >& expectedParameters, const std::vector<MatrixRotation>& optimizedRotations)
{
  assert(expectedParameters.size() == optimizedRotations.size());

  Eigen::Array<double, 3, 3> meanErrorCoeffwise = MatrixRotation::Zero();
  Eigen::Array<double, 3, 3> maxErrorCoeffwise = MatrixRotation::Zero();
  for(int i = 0; i < expectedParameters.size(); i++)
  {
    Eigen::Array<double, 3, 3> error = (expectedParameters.at(i).pose.rotation - optimizedRotations.at(i));
    meanErrorCoeffwise += error.abs();
    maxErrorCoeffwise = maxErrorCoeffwise.max(error.abs());
  }
  meanErrorCoeffwise /= expectedParameters.size();

  std::cout << "Parameter error statistics: " << std::endl;
  std::cout << "Mean error:" << std::endl << meanErrorCoeffwise << std::endl;
  std::cout << "Max error:" << std::endl << maxErrorCoeffwise << std::endl;
}

template<typename RotationType>
void optimizeImpl(const MeasuredScene<RotationType>& measurements, OptimizedScene<RotationType>& parameters)
{
  auto initialParameters = parameters;

  /*
   *  1. Setup problem
   */
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

  /*
   *  3. Output report
   */

  std::cout << summary.FullReport() << std::endl;
  std::cout << "First 10 cameras: " << std::endl;
  auto n = std::min(10lu,measurements.cameras.size());
  for(int i = 0; i < n ; i++)
  {
    print(measurements.cameras.at(i).pose.rotation, initialParameters.rotation.at(i), parameters.rotation.at(i));
  }
  printErrorStatistics(measurements.cameras, parameters.rotation);
}
} // namespace

template<> void optimize(const MeasuredScene<AngleAxisRotation>& measurements, OptimizedScene<AngleAxisRotation>& parameters)
{
  optimizeImpl(measurements, parameters);
}
template<> void optimize(const MeasuredScene<QuaternionRotation>& measurements, OptimizedScene<QuaternionRotation>& parameters)
{
  optimizeImpl(measurements, parameters);
}
template<> void optimize(const MeasuredScene<MatrixRotation>& measurements, OptimizedScene<MatrixRotation>& parameters)
{
  optimizeImpl(measurements, parameters);
}
} //namespace roto
