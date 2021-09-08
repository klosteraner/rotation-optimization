#include "geometry.hpp"

namespace sote
{
  template<>
  Eigen::Vector2d project(const CameraPose<AngleAxisRotation>& pose, const CameraSensor& sensor, const Eigen::Vector3d& worldCoordinates)
  {
    double projection[2];
    projectAngleAxis(pose.rotation.data(), pose.position, sensor, worldCoordinates, projection);
    return Eigen::Map<Eigen::Vector2d>(projection);
  }

  template<>
  Eigen::Vector2d project(const CameraPose<QuaternionRotation>& pose, const CameraSensor& sensor, const Eigen::Vector3d& worldCoordinates)
  {
    double projection[2];
    projectQuaternion(pose.rotation.coeffs().data(), pose.position, sensor, worldCoordinates, projection);
    return Eigen::Map<Eigen::Vector2d>(projection);
  }
}
