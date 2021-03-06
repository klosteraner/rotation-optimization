#include "quaternionReprojectionError.hpp"

namespace roto
{
  void J_projectQuaternion_angleAxis(const double eigenUnitQuaternionRotation[4], const Eigen::Vector3d& cameraPosition,
                                     const CameraSensor& sensor, const Eigen::Vector3d& worldCoordinates, double jacobian[6])
  {
    // Extrinsis jacobian Jworld->camera_angleAxis
    Eigen::Map<const Eigen::Quaterniond> q(eigenUnitQuaternionRotation);
    const Eigen::Vector3d p = worldCoordinates - cameraPosition;

    Eigen::Matrix<double, 3, 3> p_skew;
    p_skew << 0,      -p.z(), p.y(),
              p.z(),  0,      -p.x(),
              -p.y(), p.x(),  0;
    Eigen::Matrix<double, 3, 3> J_rotatePoint_angleAxis;
    J_rotatePoint_angleAxis = - q.toRotationMatrix() * p_skew;

    //std::cout << "skew" << p_skew << std::endl;
    //std::cout << "rotation: " << q.toRotationMatrix() << std::endl;
    //std::cout << "skew applied" << J_rotatePoint_angleAxis << std::endl;

    // Intrinsics Jacobian
    Eigen::Vector3d cameraCoordinates;
    eigenUnitQuaternionRotatePoint(eigenUnitQuaternionRotation, p.data(), cameraCoordinates.data());
    Eigen::Matrix<double, 2, 3> J_project_cameraCoordinates = compute_J_project_cameraCoordinates(cameraCoordinates, sensor);

    Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J(jacobian);
    J = J_project_cameraCoordinates * J_rotatePoint_angleAxis;
  }

  void J_projectQuaternion_q(const double eigenUnitQuaternionRotation[4], const Eigen::Vector3d& cameraPosition,
                             const CameraSensor& sensor, const Eigen::Vector3d& worldCoordinates, double jacobian[8])
  {
    // We cannot use the derivative w.r.t the lie algebra(tangent space)
    // d(R(theta)*p) / dtheta = - R * skew(p),
    // but ceres requires the derivative w.r.t. the lie group(manifold)
    // d(R(q)*p) / q instead. Since ceres chaines dq/dtheta in the LocalParametrization.
    // The computation is taken from
    // http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf
    Eigen::Map<const Eigen::Quaterniond> q(eigenUnitQuaternionRotation);
    const Eigen::Vector3d p = worldCoordinates - cameraPosition;

    Eigen::Matrix<double, 3, 4> J_Rp_q_sola;

    const double qxpx = q.x()*p.x();
    const double qxpy = q.x()*p.y();
    const double qxpz = q.x()*p.z();

    const double qypx = q.y()*p.x();
    const double qypy = q.y()*p.y();
    const double qypz = q.y()*p.z();

    const double qzpx = q.z()*p.x();
    const double qzpy = q.z()*p.y();
    const double qzpz = q.z()*p.z();

    const double qwpx = q.w()*p.x();
    const double qwpy = q.w()*p.y();
    const double qwpz = q.w()*p.z();

    const double J_Rp_q_diag = 2*(qxpx + qypy + qzpz);
    const double J_Rp_q_10 = 2*(qypx - qxpy - qwpz);
    const double J_Rp_q_20 = 2*(-qzpx + qxpz - qwpy);
    const double J_Rp_q_21 = 2*(qzpy - qypz - qwpx);

    J_Rp_q_sola <<
       J_Rp_q_diag,  -J_Rp_q_10,    J_Rp_q_20,   2*(qwpx + qypz - qzpy),
       J_Rp_q_10,     J_Rp_q_diag, -J_Rp_q_21,   2*(qwpy - qxpz + qzpx),
      -J_Rp_q_20,     J_Rp_q_21,    J_Rp_q_diag, 2*(qwpz + qxpy - qypx);

    Eigen::Vector3d cameraCoordinates;
    eigenUnitQuaternionRotatePoint(eigenUnitQuaternionRotation, p.data(), cameraCoordinates.data());
    Eigen::Matrix<double, 2, 3> J_project_cameraCoordinates = compute_J_project_cameraCoordinates(cameraCoordinates, sensor);

    // Ceres expects row major jacobian, while Eigen operates with col major by default.
    Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J(jacobian);
    J = J_project_cameraCoordinates * J_Rp_q_sola;

    if constexpr(DEBUG_LOGGING)
    {
      //********************************************************************************
      // Correctness check 1: Compare to analytical Jacobian on Lie Algebra (angleAxis):
      //********************************************************************************
      //
      // dRp / dAngleAxis =(!) (dRp / dq) * dq / dAngleAxis
      // Compute Parametrization at q using ceres convention:
      // J (exp(angleAxis) * q ) - Note the order (opposite to Sola convention)

      Eigen::Matrix<double, 4, 3> J_expoq_angleAxis;
      J_expoq_angleAxis <<
        q.w(),  q.z(), -q.y(),
       -q.z(),  q.w(),  q.x(),
        q.y(), -q.x(),  q.w(),
       -q.x(), -q.y(), -q.z();
      Eigen::Matrix<double, 3, 3> J_Rp_angleAxis_chained = J_Rp_q_sola * J_expoq_angleAxis;
      std::cout << "Chained: J_Rp_q_sola * J_expoq_angleAxis" << std::endl;
      std::cout << J_Rp_angleAxis_chained(0,0) << ", " << J_Rp_angleAxis_chained(0,1) << ", " << J_Rp_angleAxis_chained(0,2) << std::endl;
      std::cout << J_Rp_angleAxis_chained(1,0) << ", " << J_Rp_angleAxis_chained(1,1) << ", " << J_Rp_angleAxis_chained(1,2) << std::endl;
      std::cout << J_Rp_angleAxis_chained(2,0) << ", " << J_Rp_angleAxis_chained(2,1) << ", " << J_Rp_angleAxis_chained(2,2) << std::endl;

      // Here for exp(angleAxis) * q -> - 2 * [R*p_world]_x
      // Note again difference to Sola: For q * exp(angleAxis) -> - R * [p_world]_x
      // Note also: Factor 2 originates from Ceres non-standard parameterization
      const Eigen::Vector3d& p_w = cameraCoordinates;
      Eigen::Matrix<double, 3, 3> p_skew;
      p_skew <<
         0,       -p_w.z(),  p_w.y(),
         p_w.z(),  0,       -p_w.x(),
        -p_w.y(),  p_w.x(),  0;
      Eigen::Matrix<double, 3, 3> J_Rp_angleAxis = -2 * p_skew;
      std::cout << "J_Rp_angleAxis:" << std::endl;
      std::cout << J_Rp_angleAxis(0,0) << ", " << J_Rp_angleAxis(0,1) << ", " << J_Rp_angleAxis(0,2) << std::endl;
      std::cout << J_Rp_angleAxis(1,0) << ", " << J_Rp_angleAxis(1,1) << ", " << J_Rp_angleAxis(1,2) << std::endl;
      std::cout << J_Rp_angleAxis(2,0) << ", " << J_Rp_angleAxis(2,1) << ", " << J_Rp_angleAxis(2,2) << std::endl;

      //********************************************************************************
      // Correctness check 2: Compare to numerical Jacobian on Lie Algebra (angleAxis):
      //********************************************************************************
      const double delta = 1e-6;
      const Eigen::Vector3d x_plus_dx(delta, 0, 0);
      const Eigen::Vector3d x_plus_dy(0, delta, 0);
      const Eigen::Vector3d x_plus_dz(0, 0, delta);

      // Note that Eigen uses standard exponential map, while ceres does not.
      const Eigen::Quaterniond q_plus_dx = Eigen::Quaterniond(Eigen::AngleAxisd(2.*x_plus_dx.norm(), x_plus_dx/x_plus_dx.norm())) * q;
      const Eigen::Quaterniond q_plus_dy = (Eigen::AngleAxisd(2.*x_plus_dy.norm(), x_plus_dy/x_plus_dy.norm())) * q;
      const Eigen::Quaterniond q_plus_dz = (Eigen::AngleAxisd(2.*x_plus_dz.norm(), x_plus_dz/x_plus_dz.norm())) * q;

      Eigen::Vector3d f, f_plus_dx, f_plus_dy, f_plus_dz;
      compute_cameraCoordinatesQuaternion(q.coeffs().data(),          worldCoordinates, cameraPosition, f.data());
      compute_cameraCoordinatesQuaternion(q_plus_dx.coeffs().data(),  worldCoordinates, cameraPosition, f_plus_dx.data());
      compute_cameraCoordinatesQuaternion(q_plus_dy.coeffs().data(),  worldCoordinates, cameraPosition, f_plus_dy.data());
      compute_cameraCoordinatesQuaternion(q_plus_dz.coeffs().data(),  worldCoordinates, cameraPosition, f_plus_dz.data());

      const Eigen::Vector3d fdx_p = (f_plus_dx - f)/ delta;
      const Eigen::Vector3d fdy_p = (f_plus_dy - f)/ delta;
      const Eigen::Vector3d fdz_p = (f_plus_dz - f)/ delta;
      Eigen::Matrix<double, 3, 3> numerical_J_project_angleAxis_plus;
      numerical_J_project_angleAxis_plus << fdx_p(0), fdy_p(0), fdz_p(0),
                                            fdx_p(1), fdy_p(1), fdz_p(1),
                                            fdx_p(2), fdy_p(2), fdz_p(2);
      std::cout << "Numerical external Jac: " << std::endl;
      std::cout << numerical_J_project_angleAxis_plus << std::endl;
    }
  }

bool QuaternionReprojectionCostFunctionAna::Evaluate(double const* const* eigenUnitQuaternionRotation,
                                                     double* residuals, double** jacobians) const
{
  Eigen::Map<const Eigen::Quaterniond> q(eigenUnitQuaternionRotation[0]);

  double projection[2];
  projectQuaternion(eigenUnitQuaternionRotation[0], cameraPosition, sensor, track3D, projection);
  residuals[0] = projection[0] - mark(0);
  residuals[1] = projection[1] - mark(1);

  if(jacobians != nullptr && jacobians[0] != nullptr)
  {
    // Some calculations are done again: Ok for now.
    J_projectQuaternion_q(eigenUnitQuaternionRotation[0], cameraPosition, sensor, track3D, jacobians[0]);
  }

  return true;
}

bool LieStyleQuaternionReprojectionCostFunctionAna::Evaluate(double const* const* eigenUnitQuaternionRotation,
                                                             double* residuals, double** jacobians) const
{
  Eigen::Map<const QuaternionRotation> q(eigenUnitQuaternionRotation[0]);

  // This is actually a matrix rotation operation:
  // R(q)*(track3D - cameraPosition)
  // with a few multiplications to setup R(q)
  const Eigen::Vector3d cameraCoordinates = q * (track3D - cameraPosition);

  const double f_by_z = sensor.f / cameraCoordinates.z();
  residuals[0] = (-cameraCoordinates.x() * f_by_z + sensor.cx) - mark.x();
  residuals[1] = (-cameraCoordinates.y() * f_by_z + sensor.cy) - mark.y();

  if(jacobians != nullptr && jacobians[0] != nullptr)
  {
    const double f_by_zz = f_by_z / cameraCoordinates.z();

    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_project_cameraCoordinates;
    J_project_cameraCoordinates << -f_by_z, 0, cameraCoordinates.x() * f_by_zz,
                                    0, -f_by_z, cameraCoordinates.y() * f_by_zz;

    // We pretend to use the quaternion jacobian J_project_q,
    Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J(jacobians[0]);
    // while in fact we fill in the locally parameterized
    // J_project_angleAxis = J_project_q * J_q_angleAxis
    // = J_intrinsics * J_extrinsics
    // = J_project_cameraCoordinates * (-[cameraCoordinates]_x)
    // where [p]_x is a skew matrix [ 0,  -p_z,  p_y,
    //                                p_z, 0,   -p_x,
    //                               -p_y, p_x,  0  ]
    // and v * [p]_x = v x p = - p x v, the crossproduct
    J.block<1,3>(0,0) = cameraCoordinates.cross(J_project_cameraCoordinates.row(0));
    J.block<1,3>(1,0) = cameraCoordinates.cross(J_project_cameraCoordinates.row(1));
    J.col(3).setZero();
  }
  return true;
}

/// This is a mock jacobian: identity on first 3 elements (tangent space), 0 on last
bool LieStyleParameterization::ComputeJacobian(const double* q, double* J_q_delta_at_zero) const
{
  Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> J(J_q_delta_at_zero);
  J.block<3,3>(0,0).setIdentity();
  J.block<1,3>(3,0).setZero();
  return true;
}

void addQuaternionReprojectionError(ceres::Problem& problem, const MeasuredScene<QuaternionRotation>& measurements,
                                    OptimizedScene<QuaternionRotation>& parameters, DerivativeType derivativeType)
{
  for(const auto& track : measurements.tracks)
  {
    for(const auto& camIdAndMark : track.marks)
    {
      const auto& camera = measurements.cameras.at(camIdAndMark.first);

      ceres::CostFunction* f;
      switch(derivativeType)
      {
        case DerivativeType::Automatic:
          f = new ceres::AutoDiffCostFunction<QuaternionReprojectionCostFunctorAuto, 2, 4>(
                new QuaternionReprojectionCostFunctorAuto(camera.pose.position,
                                                          measurements.cameraSensors.at(camera.sensorId),
                                                          camIdAndMark.second,
                                                          track.originalPosition));
          // Parametrization via exp(delta) * q (without scaling, i.e. delta = 2*angleAxis)
          problem.AddParameterBlock(parameters.rotation[camIdAndMark.first].coeffs().data(),
                                    4, new ceres::EigenQuaternionParameterization());

          break;
        case DerivativeType::AnalyticCeresStyle:
          f = new QuaternionReprojectionCostFunctionAna(camera.pose.position,
                                                        measurements.cameraSensors.at(camera.sensorId),
                                                        camIdAndMark.second,
                                                        track.originalPosition);
          // Parametrization via exp(delta) * q (without scaling, i.e. delta = 2*angleAxis)
          problem.AddParameterBlock(parameters.rotation[camIdAndMark.first].coeffs().data(),
                                    4, new ceres::EigenQuaternionParameterization());
          break;
        case DerivativeType::AnalyticLieStyle:
          f = new LieStyleQuaternionReprojectionCostFunctionAna(camera.pose.position,
                                                                measurements.cameraSensors.at(camera.sensorId),
                                                                camIdAndMark.second,
                                                                track.originalPosition);
          // Mock Parametrization (actual parametrization in cost function, same parametrizatoin as the other)
          problem.AddParameterBlock(parameters.rotation[camIdAndMark.first].coeffs().data(),
                                    4, new LieStyleParameterization());
      }

      problem.AddResidualBlock(f, nullptr, parameters.rotation[camIdAndMark.first].coeffs().data());
    }
  }
}
} // namespace roto
