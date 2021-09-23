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

  Eigen::Matrix<double, 2, 3> compute_J_project_cameraCoordinates(const Eigen::Vector3d& cameraCoordinates, const CameraSensor& sensor)
  {
    const double f_by_z = - sensor.f / cameraCoordinates.z();
    const double f_by_zz = - f_by_z / cameraCoordinates.z();
    Eigen::Matrix<double, 2, 3> J;
    J <<  f_by_z, 0,      cameraCoordinates.x() * f_by_zz,
          0,      f_by_z, cameraCoordinates.y() * f_by_zz;
    return J;
  }

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

    std::cout << "skew" << p_skew << std::endl;
    std::cout << "rotation: " << q.toRotationMatrix() << std::endl;
    std::cout << "skew applied" << J_rotatePoint_angleAxis << std::endl;

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

    double qxpy = q.x()*p.y();
    double qxpz = q.x()*p.z();
    double qypx = q.y()*p.x();
    double qypz = q.y()*p.z();
    double qzpx = q.z()*p.x();
    double qzpy = q.z()*p.y();
    double qwpx = q.w()*p.x();
    double qwpy = q.w()*p.y();
    double qwpz = q.w()*p.z();
    double jRDiag = 2*(q.x()*p.x() + q.y()*p.y() + q.z()*p.z());

    Eigen::Matrix<double, 3, 4> J_rotatePoint_q;
    J_rotatePoint_q <<  jRDiag,                 2*(qypx - qxpy - qwpz), 2*(qzpx - qxpz + qwpy), 2*(qwpx + qypz - qzpy),
                        2*(qxpy - qypx + qwpz), jRDiag,                 2*(qzpy - qypz - qwpx), 2*(qwpy - qxpz + qzpx),
                        2*(qxpz - qzpx - qwpy), 2*(qypz - qzpy + qwpx), jRDiag,                 2*(qwpz + qxpy - qypx);

    std::cout << "External Jac:" << std::endl;
    std::cout << J_rotatePoint_q(0,0) << ", " << J_rotatePoint_q(0,1) << ", " << J_rotatePoint_q(0,2) << ", " << J_rotatePoint_q(0,3) << std::endl;
    std::cout << J_rotatePoint_q(1,0) << ", " << J_rotatePoint_q(1,1) << ", " << J_rotatePoint_q(1,2) << ", " << J_rotatePoint_q(1,3) << std::endl;
    std::cout << J_rotatePoint_q(2,0) << ", " << J_rotatePoint_q(2,1) << ", " << J_rotatePoint_q(2,2) << ", " << J_rotatePoint_q(2,3) << std::endl;

    Eigen::Vector3d cameraCoordinates;
    eigenUnitQuaternionRotatePoint(eigenUnitQuaternionRotation, p.data(), cameraCoordinates.data());
    Eigen::Matrix<double, 2, 3> J_project_cameraCoordinates = compute_J_project_cameraCoordinates(cameraCoordinates, sensor);

    std::cout << "J_external_angleaxis using ceres param" << std::endl;
    Eigen::Matrix<double, 4, 3> J_q_angleAxis;
    J_q_angleAxis << q.coeffs().data()[3],  q.coeffs().data()[2], -q.coeffs().data()[1],
                    -q.coeffs().data()[2],  q.coeffs().data()[3],  q.coeffs().data()[0],
                     q.coeffs().data()[1], -q.coeffs().data()[0],  q.coeffs().data()[3],
                    -q.coeffs().data()[0], -q.coeffs().data()[1], -q.coeffs().data()[2];
    std::cout << "J_q_angleAxis: " << J_q_angleAxis << std::endl;
    std::cout << "J_external_angleAxis: " << J_rotatePoint_q * J_q_angleAxis << std::endl;

    std::cout << "Numerical external Jac: " << std::endl;

    const double delta = 1e-5;
    Eigen::AngleAxisd angleAxis(q);
    Eigen::Vector3d x = angleAxis.angle() * angleAxis.axis();
    Eigen::Vector3d x_plus_dx(x.x()+delta, x.y(),       x.z());
    Eigen::Vector3d x_plus_dy(x.x(),       x.y()+delta, x.z());
    Eigen::Vector3d x_plus_dz(x.x(),       x.y(),       x.z()+delta);

    Eigen::Quaterniond q_plus_dx(Eigen::AngleAxisd(x_plus_dx.norm(), x_plus_dx/x_plus_dx.norm()));
    Eigen::Quaterniond q_plus_dy(Eigen::AngleAxisd(x_plus_dy.norm(), x_plus_dy/x_plus_dy.norm()));
    Eigen::Quaterniond q_plus_dz(Eigen::AngleAxisd(x_plus_dz.norm(), x_plus_dz/x_plus_dz.norm()));

    Eigen::Vector3d f, f_plus_dx, f_plus_dy, f_plus_dz;
    compute_cameraCoordinates(q.coeffs().data(),          worldCoordinates, cameraPosition, f.data());
    compute_cameraCoordinates(q_plus_dx.coeffs().data(),  worldCoordinates, cameraPosition, f_plus_dx.data());
    compute_cameraCoordinates(q_plus_dy.coeffs().data(),  worldCoordinates, cameraPosition, f_plus_dy.data());
    compute_cameraCoordinates(q_plus_dz.coeffs().data(),  worldCoordinates, cameraPosition, f_plus_dz.data());

    Eigen::Vector3d fdx_p = (f_plus_dx - f)/ delta;
    Eigen::Vector3d fdy_p = (f_plus_dy - f)/ delta;
    Eigen::Vector3d fdz_p = (f_plus_dz - f)/ delta;
    Eigen::Matrix<double, 3, 3> numerical_J_project_angleAxis_plus;
    numerical_J_project_angleAxis_plus << fdx_p(0), fdy_p(0), fdz_p(0),
                                          fdx_p(1), fdy_p(1), fdz_p(1),
                                          fdx_p(2), fdy_p(2), fdz_p(2);
    std::cout << numerical_J_project_angleAxis_plus << std::endl;

    // Ceres expects row major jacobian, while Eigen operates with col major by default.
    Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J(jacobian);
    J = J_project_cameraCoordinates * J_rotatePoint_q;
  }
}
