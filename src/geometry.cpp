#include "geometry.hpp"

namespace roto
{
Eigen::Matrix<double, 2, 3> compute_J_project_cameraCoordinates(const Eigen::Vector3d& cameraCoordinates, const CameraSensor& sensor)
{
  const double f_by_z = - sensor.f / cameraCoordinates.z();
  const double f_by_zz = - f_by_z / cameraCoordinates.z();
  Eigen::Matrix<double, 2, 3> J;
  J <<  f_by_z, 0,      cameraCoordinates.x() * f_by_zz,
        0,      f_by_z, cameraCoordinates.y() * f_by_zz;
  return J;
}
} // namespace roto
