#pragma once

#include "types.h"

#include <ceres/rotation.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sote
{
  // Projection function to be used everywhere
  // including ceres (that's why the templating)
  template<typename T>
  bool project(const T* angleAxisRotation, const Eigen::Vector3d& cameraPosition, const CameraSensor& sensor, const Eigen::Vector3d& worldCoordinates, T* projection)
  {
    // 1. Apply extrinsics: world -> camera coordinates
    const T p[3] = {T(worldCoordinates[0] - cameraPosition[0]),
                    T(worldCoordinates[1] - cameraPosition[1]),
                    T(worldCoordinates[2] - cameraPosition[2])};
    T p_camera[3];
    ceres::AngleAxisRotatePoint(angleAxisRotation, p, p_camera);

    // 2. Project
    const T f_by_z = - T(sensor.f) / p_camera[2];
    projection[0] = p_camera[0] * f_by_z + T(sensor.cx);
    projection[1] = p_camera[1] * f_by_z + T(sensor.cy);

    return true;
  }

  // Convenience overload
  Eigen::Vector2d project(const CameraPose& pose, const CameraSensor& sensor, const Eigen::Vector3d& worldCoordinates);
}
