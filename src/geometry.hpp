#pragma once

#include "types.h"

#include <ceres/rotation.h>

namespace roto
{
/*
 *  Camera intrinsics
 */

template<typename T>
void project(const T cameraCoordinates[3], const CameraSensor& sensor, T projection[2])
{
  const T f_by_z = - T(sensor.f) / cameraCoordinates[2];
  projection[0] = cameraCoordinates[0] * f_by_z + T(sensor.cx);
  projection[1] = cameraCoordinates[1] * f_by_z + T(sensor.cy);
}

Eigen::Matrix<double, 2, 3> compute_J_project_cameraCoordinates(const Eigen::Vector3d& cameraCoordinates, const CameraSensor& sensor);

// Adapted from ceres using eigen parameter order
template<typename T>
inline void eigenUnitQuaternionRotatePoint(const T eigenQ[4],
                                           const T pt[3], T result[3])
{
  DCHECK_NE(pt, result) << "Inplace rotation is not supported.";

  T uv0 = eigenQ[1] * pt[2] - eigenQ[2] * pt[1];
  T uv1 = eigenQ[2] * pt[0] - eigenQ[0] * pt[2];
  T uv2 = eigenQ[0] * pt[1] - eigenQ[1] * pt[0];
  uv0 += uv0;
  uv1 += uv1;
  uv2 += uv2;
  result[0] = pt[0] + eigenQ[3] * uv0 + (eigenQ[1] * uv2 - eigenQ[2] * uv1);
  result[1] = pt[1] + eigenQ[3] * uv1 + (eigenQ[2] * uv0 - eigenQ[0] * uv2);
  result[2] = pt[2] + eigenQ[3] * uv2 + (eigenQ[0] * uv1 - eigenQ[1] * uv0);
}
} // namespace roto
