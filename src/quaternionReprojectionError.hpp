#pragma once

#include "types.h"
#include "geometry.hpp"

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace roto
{
template<typename T>
void compute_cameraCoordinatesQuaternion(const T eigenQ[4], const Eigen::Vector3d& worldCoordinates,
                                         const Eigen::Vector3d& cameraPosition, T result[3])
{
  // 1. Apply extrinsics: world -> camera coordinates
  const T p[3] = {T(worldCoordinates[0] - cameraPosition[0]),
                  T(worldCoordinates[1] - cameraPosition[1]),
                  T(worldCoordinates[2] - cameraPosition[2])};
  eigenUnitQuaternionRotatePoint(eigenQ, p, result);
}

// Projection function to be used everywhere
// including ceres (that's why the templating)
template<typename T>
void projectQuaternion(const T eigenUnitQuaternionRotation[4], const Eigen::Vector3d& cameraPosition,
                       const CameraSensor& sensor, const Eigen::Vector3d& worldCoordinates, T projection[2])
{
  // 1. Apply extrinsics: world -> camera coordinates
  T p_camera[3];
  compute_cameraCoordinatesQuaternion(eigenUnitQuaternionRotation, worldCoordinates, cameraPosition, p_camera);

  // 2. Apply intrinsics
  project(p_camera, sensor, projection);
}

// Analytic derivative of the projectQuaternion function
// Since its already the derivative, there is no need to support jets
void J_projectQuaternion_q(const double eigenUnitQuaternionRotation[4], const Eigen::Vector3d& cameraPosition,
                           const CameraSensor& sensor, const Eigen::Vector3d& worldCoordinates, double jacobian[8]);

void J_projectQuaternion_angleAxis(const double eigenUnitQuaternionRotation[4], const Eigen::Vector3d& cameraPosition,
                                   const CameraSensor& sensor, const Eigen::Vector3d& worldCoordinates, double jacobian[6]);


/// cost function for automatic derivative
/// using local angle-axis parametrization (Singularity-free)
struct QuaternionReprojectionCostFunctorAuto
{
  QuaternionReprojectionCostFunctorAuto(const Eigen::Vector3d& _cameraPosition, const CameraSensor& _sensor,
                                              const Eigen::Vector2d& _mark, const Eigen::Vector3d& _track3D)
  : cameraPosition(_cameraPosition),
    sensor(_sensor),
    mark(_mark),
    track3D(_track3D){}

  template <typename T>
  bool operator()(const T* quaternionRotation, T* residuals) const
  {
    T projection[2];
    projectQuaternion(quaternionRotation, cameraPosition, sensor, track3D, projection);
    residuals[0] = projection[0] - T(mark(0));
    residuals[1] = projection[1] - T(mark(1));

    return true;
  }

private:
  const Eigen::Vector3d& cameraPosition;
  const CameraSensor& sensor;
  const Eigen::Vector2d& mark;
  const Eigen::Vector3d& track3D;
};

/// Cost function analytical derivative
/// using local angle-axis parametrization (Singularity-free)
struct QuaternionReprojectionCostFunctionAna : public ceres::SizedCostFunction<2, 4>
{
  QuaternionReprojectionCostFunctionAna(const Eigen::Vector3d& _cameraPosition, const CameraSensor& _sensor,
                                        const Eigen::Vector2d& _mark, const Eigen::Vector3d& _track3D)
  : cameraPosition(_cameraPosition),
    sensor(_sensor),
    mark(_mark),
    track3D(_track3D){}

  bool Evaluate(double const* const* eigenUnitQuaternionRotation, double* residuals, double** jacobians) const;

private:
  const Eigen::Vector3d& cameraPosition;
  const CameraSensor& sensor;
  const Eigen::Vector2d& mark;
  const Eigen::Vector3d& track3D;
};

void addQuaternionReprojectionError(ceres::Problem& problem, const MeasuredScene<QuaternionRotation>& measurements,
                                    OptimizedScene<QuaternionRotation>& parameters, bool useAnalyticDerivative = true);
} // namespace roto
