#include "matrixReprojectionError.hpp"

namespace roto
{
// Analytic derivative of the projectMatrix function
// Since its already the derivative, there is no need to support jets
void J_projectMatrix_R(const double matrixRotation[9], const Eigen::Vector3d& cameraPosition,
                       const CameraSensor& sensor, const Eigen::Vector3d& worldCoordinates, double jacobian[18])
{
  Eigen::Map<const Eigen::Matrix3d> R(matrixRotation);
  const Eigen::Vector3d p = worldCoordinates - cameraPosition;
  const Eigen::Vector3d p_cam = R*p;

  // Directly construct dproj(Rx)/dR = dproj(p)/dp * dRx/dR
  // dproj(p)/dp =  (-f/z 0   x*f/(z*z),
  //                 0    f/z y*f/(z*z))
  // dRx / dR = ( px py pz 0  0  0  0  0  0
  //              0  0  0  px py pz 0  0  0
  //              0  0  0  0  0  0  px py pz)

  const double by_z = - 1 / p_cam.z();
  const double f_by_z = sensor.f * by_z;

  const double fpx_by_z = p.x() * f_by_z;
  const double fpy_by_z = p.y() * f_by_z;
  const double fpz_by_z = p.z() * f_by_z;

  const double f_by_zz = f_by_z * by_z;
  const double fx_by_zz = p_cam.x() * f_by_zz;
  const double fy_by_zz = p_cam.y() * f_by_zz;

  // Ceres expects row major jacobian, while Eigen operates with col major by default.
  Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor>> J(jacobian);
  J <<  fpx_by_z, fpy_by_z, fpz_by_z, 0, 0, 0,
        p.x()*fx_by_zz, p.y()*fx_by_zz, p.z()*fx_by_zz, // end first row dpr_x(R*p)/dR

        0, 0, 0, fpx_by_z, fpy_by_z, fpz_by_z,
        p.x()*fy_by_zz, p.y()*fy_by_zz, p.z()*fy_by_zz; // end second row dpr_x(R*p)/dR
}

bool RotationMatrixReprojectionCostFunctionAna::Evaluate(double const* const* rotationMatrix,
                                                         double* residuals, double** jacobians) const
{
  Eigen::Map<const MatrixRotation> R(rotationMatrix[0]);

  Eigen::Vector2d projection;
  projectMatrix(R.data(), cameraPosition, sensor, track3D, projection.data());

  residuals[0] = projection[0] - mark(0);
  residuals[1] = projection[1] - mark(1);

  if(jacobians != nullptr && jacobians[0] != nullptr)
  {
    // Some calculations are done again: Ok for now.
    J_projectMatrix_R(rotationMatrix[0], cameraPosition, sensor, track3D, jacobians[0]);
  }

  return true;
}

bool RotationMatrixReprojectionCostFunctionAna2::Evaluate(double const* const* rowMajorRotationMatrix,
                                                          double* residuals, double** jacobians) const
{
  Eigen::Map<const MatrixRotation> R(rowMajorRotationMatrix[0]);
  const Eigen::Vector3d p = track3D - cameraPosition;
  const Eigen::Vector3d p_cam = R*p;

  const double by_z = - 1 / p_cam.z();
  const double f_by_z = sensor.f * by_z;

  residuals[0] = (p_cam.x()*f_by_z + sensor.cx) - mark(0);
  residuals[1] = (p_cam.y()*f_by_z + sensor.cy) - mark(1);

  if(jacobians != nullptr && jacobians[0] != nullptr)
  {
    const double fpx_by_z = p.x() * f_by_z;
    const double fpy_by_z = p.y() * f_by_z;
    const double fpz_by_z = p.z() * f_by_z;

    const double f_by_zz = f_by_z * by_z;
    const double fx_by_zz = p_cam.x() * f_by_zz;
    const double fy_by_zz = p_cam.y() * f_by_zz;

    Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor>> J(jacobians[0]);
    J <<  fpx_by_z,
          fpy_by_z,
          fpz_by_z,
          0,
          0,
          0,
          p.x()*fx_by_zz,
          p.y()*fx_by_zz,
          p.z()*fx_by_zz, // end first row dpr_x(R*p)/dR

          0,
          0,
          0,
          fpx_by_z,
          fpy_by_z,
          fpz_by_z,
          p.x()*fy_by_zz,
          p.y()*fy_by_zz,
          p.z()*fy_by_zz; // end second row dpr_x(R*p)/dR
  }

  return true;
}

// Parametrization at R
// Note that this plus chains in the opposite order than Ceres
// Parametrizations. It follows instead the convention used in
// https://arxiv.org/pdf/1812.01537.pdf (Joan Sola)
bool RotationMatrixParameterization::Plus(const double* R, const double* delta, double* R_plus_delta) const
{
  Eigen::Map<const Eigen::Vector3d>_delta(delta);

  const double angle = _delta.norm();
  const Eigen::Vector3d axis = _delta / angle;
  const double cos = std::cos(angle);
  const double one_minus_cos = 1 - cos;
  const double sin = std::sin(angle);

  MatrixRotation Rdelta;
  Rdelta << axis.x()*axis.x()*one_minus_cos + cos,
            axis.x()*axis.y()*one_minus_cos - axis.z()*sin,
            axis.x()*axis.z()*one_minus_cos + axis.y()*sin,

            axis.y()*axis.x()*one_minus_cos + axis.z()*sin,
            axis.y()*axis.y()*one_minus_cos + cos,
            axis.y()*axis.z()*one_minus_cos - axis.x()*sin,

            axis.z()*axis.x()*one_minus_cos - axis.y()*sin,
            axis.z()*axis.y()*one_minus_cos + axis.x()*sin,
            axis.z()*axis.z()*one_minus_cos + cos;

  Eigen::Map<const MatrixRotation> _R(R);
  Eigen::Map<MatrixRotation> _R_plus_delta(R_plus_delta);
  _R_plus_delta = _R * Rdelta;

  return true;
}

bool RotationMatrixParameterization::ComputeJacobian(const double* R, double* J_R_delta_at_zero) const
{
  // We assume RowMajor R
  // + Ceres assumes RowMajor for J (!?)
  Eigen::Map<const MatrixRotation> _R(R);
  Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> J(J_R_delta_at_zero);
  J <<  0,       -_R(0,2),  _R(0,1),
        _R(0,2),  0,       -_R(0,0),
       -_R(0,1),  _R(0,0),  0,

        0,       -_R(1,2),  _R(1,1),
        _R(1,2),  0,       -_R(1,0),
       -_R(1,1),  _R(1,0),  0,

        0,       -_R(2,2),  _R(2,1),
        _R(2,2),  0,       -_R(2,0),
       -_R(2,1),  _R(2,0),  0;

  return true;
}

void addMatrixReprojectionError(ceres::Problem& problem, const MeasuredScene<MatrixRotation>& measurements,
                                OptimizedScene<MatrixRotation>& parameters, bool useAnalyticDerivative)
{
  for(const auto& track : measurements.tracks)
  {
    for(const auto& camIdAndMark : track.marks)
    {
      const auto& camera = measurements.cameras.at(camIdAndMark.first);

      ceres::CostFunction* f;
      if(useAnalyticDerivative)
      {
        // Both analytical version work the same,
        // I keep them both to have a way showing
        // the "optimization" has no significant effect
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
} // namespace roto
