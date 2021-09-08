#include "geometry.hpp"

namespace sote
{
  Eigen::Vector2d project(const CameraPose& pose, const CameraSensor& sensor, const Eigen::Vector3d& worldCoordinates)
  {
    double projection[2];
    project(pose.rotation.data(), pose.position, sensor, worldCoordinates, projection);
    return Eigen::Map<Eigen::Vector2d>(projection);
  }
}
