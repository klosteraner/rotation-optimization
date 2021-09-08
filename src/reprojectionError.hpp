#pragma once

#include "types.h"
#include "geometry.hpp"

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace sote
{
  /// Cost function for automatic derivative
  struct AngleAxisReprojectionCostFunctorAuto
  {
    AngleAxisReprojectionCostFunctorAuto(const Eigen::Vector3d& _cameraPosition, const CameraSensor& _sensor, const Eigen::Vector2d& _mark, const Eigen::Vector3d& _track3D)
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

  /// Cost function for automatic derivative
  struct QuaternionReprojectionCostFunctorAuto
  {
    QuaternionReprojectionCostFunctorAuto(const Eigen::Vector3d& _cameraPosition, const CameraSensor& _sensor, const Eigen::Vector2d& _mark, const Eigen::Vector3d& _track3D)
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
}
