#pragma once

#include "types.h"
#include "geometry.hpp"

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace roto
{
// Projection based on Ceres' (global!) angleAxis implementation
// Note: Scaling is different than for Ceres' parametrization
// used to locally parametrize R = q (Quaternion rotation)
template<typename T>
void projectAngleAxis(const T angleAxisRotation[3], const Eigen::Vector3d& cameraPosition,
                      const CameraSensor& sensor, const Eigen::Vector3d& worldCoordinates, T projection[2])
{
  // 1. Apply extrinsics: world -> camera coordinates
  const T p[3] = {T(worldCoordinates[0] - cameraPosition[0]),
                  T(worldCoordinates[1] - cameraPosition[1]),
                  T(worldCoordinates[2] - cameraPosition[2])};
  T p_camera[3];
  ceres::AngleAxisRotatePoint(angleAxisRotation, p, p_camera);

  // 2. Apply intrinsics
  project(p_camera, sensor, projection);
}

/// Cost function for automatic derivative
/// using global angle-axis parametrization
struct GlobalAngleAxisReprojectionCostFunctorAuto
{
  GlobalAngleAxisReprojectionCostFunctorAuto(const Eigen::Vector3d& _cameraPosition, const CameraSensor& _sensor, const Eigen::Vector2d& _mark, const Eigen::Vector3d& _track3D)
  : cameraPosition(_cameraPosition),
    sensor(_sensor),
    mark(_mark),
    track3D(_track3D){}

  template <typename T>
  bool operator()(const T* angleAxisRotation, T* residuals) const
  {
    T projection[2];
    projectAngleAxis(angleAxisRotation, cameraPosition, sensor, track3D, projection);
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

void addAngleAxisReprojectionError(ceres::Problem& problem, const MeasuredScene<AngleAxisRotation>& measurements, OptimizedScene<AngleAxisRotation>& parameters);
} // namespace roto
