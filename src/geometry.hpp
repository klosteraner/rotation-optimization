#pragma once

#include "types.h"

#include <ceres/rotation.h>

namespace sote
{
  template<typename T>
  void project(const T cameraCoordinates[3], const CameraSensor& sensor, T projection[2])
  {
    const T f_by_z = - T(sensor.f) / cameraCoordinates[2];
    projection[0] = cameraCoordinates[0] * f_by_z + T(sensor.cx);
    projection[1] = cameraCoordinates[1] * f_by_z + T(sensor.cy);
  }

  Eigen::Matrix<double, 2, 3> compute_J_project_cameraCoordinates(const Eigen::Vector3d& cameraCoordinates, const CameraSensor& sensor);

  // Projection function to be used everywhere
  // including ceres (that's why the templating)
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

  template<typename T>
  void compute_cameraCoordinates(const T eigenQ[4], const Eigen::Vector3d& worldCoordinates, const Eigen::Vector3d& cameraPosition, T result[3])
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
    compute_cameraCoordinates(eigenUnitQuaternionRotation, worldCoordinates, cameraPosition, p_camera);

    // 2. Apply intrinsics
    project(p_camera, sensor, projection);
  }

  // Analytic derivative of the projectQuaternion function
  // Since its already the derivative, there is no need to support jets
  void J_projectQuaternion_q(const double eigenUnitQuaternionRotation[4], const Eigen::Vector3d& cameraPosition,
                             const CameraSensor& sensor, const Eigen::Vector3d& worldCoordinates, double jacobian[8]);

  void J_projectQuaternion_angleAxis(const double eigenUnitQuaternionRotation[4], const Eigen::Vector3d& cameraPosition,
                                     const CameraSensor& sensor, const Eigen::Vector3d& worldCoordinates, double jacobian[6]);

  // Convenience overload
  template<typename RotationType>
  Eigen::Vector2d project(const CameraPose<RotationType>& pose, const CameraSensor& sensor, const Eigen::Vector3d& worldCoordinates);
}
