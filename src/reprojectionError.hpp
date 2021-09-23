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

  struct QuaternionReprojectionCostFunctionAna : public ceres::SizedCostFunction<2, 4>
  {
    QuaternionReprojectionCostFunctionAna(const Eigen::Vector3d& _cameraPosition, const CameraSensor& _sensor, const Eigen::Vector2d& _mark, const Eigen::Vector3d& _track3D)
    : cameraPosition(_cameraPosition),
      sensor(_sensor),
      mark(_mark),
      track3D(_track3D){}

    bool Evaluate(double const* const* eigenUnitQuaternionRotation, double* residuals, double** jacobians) const
    {
      Eigen::Map<const Eigen::Quaterniond> q(eigenUnitQuaternionRotation[0]);

      double projection[2];
      projectQuaternion(eigenUnitQuaternionRotation[0], cameraPosition, sensor, track3D, projection);
      residuals[0] = projection[0] - mark(0);
      residuals[1] = projection[1] - mark(1);

      if(jacobians != nullptr && jacobians[0] != nullptr)
      {
        // Do the full Jacobian, and undo the last parametrization jacobian
        std::cout << "Original Ana Jac:" << std::endl;
        J_projectQuaternion_angleAxis(eigenUnitQuaternionRotation[0], cameraPosition, sensor, track3D, jacobians[0]);

        std::cout << jacobians[0][0] << ", " << jacobians[0][1] << ", " << jacobians[0][2] << std::endl;
        std::cout << jacobians[0][3] << ", " << jacobians[0][4] << ", " << jacobians[0][5] << std::endl;

        // No extra calculation overhead
        J_projectQuaternion_q(eigenUnitQuaternionRotation[0], cameraPosition, sensor, track3D, jacobians[0]);

        std::cout << "Ana Jac:" << std::endl;
        std::cout << jacobians[0][0] << ", " << jacobians[0][1] << ", " << jacobians[0][2] << ", " << jacobians[0][3] << std::endl;
        std::cout << jacobians[0][4] << ", " << jacobians[0][5] << ", " << jacobians[0][6] << ", " << jacobians[0][7] << std::endl;

        //double next_projection;
        //projectionQuaternion()

        // Apply localizatoin jacobian
        Eigen::Matrix<double, 4, 3> J_q_angleAxis;
        J_q_angleAxis << q.coeffs().data()[3],  q.coeffs().data()[2], -q.coeffs().data()[1],
                        -q.coeffs().data()[2],  q.coeffs().data()[3],  q.coeffs().data()[0],
                         q.coeffs().data()[1], -q.coeffs().data()[0],  q.coeffs().data()[3],
                        -q.coeffs().data()[0], -q.coeffs().data()[1], -q.coeffs().data()[2];
        Eigen::Map<const Eigen::Matrix<double, 2, 4>> J_project_q(jacobians[0]);
        const Eigen::Matrix<double, 2, 3> J_project_angleAxis = J_project_q * J_q_angleAxis;

        std::cout << "Localized jacobian" << std::endl;
        std::cout << J_project_angleAxis(0,0) << ", " << J_project_angleAxis(0,1) << ", " << J_project_angleAxis(0,2) << std::endl;
        std::cout << J_project_angleAxis(1,0) << ", " << J_project_angleAxis(1,1) << ", " << J_project_angleAxis(1,2) << std::endl;

        std::cout << "Numerical jacobian" << std::endl;
        Eigen::AngleAxisd angleAxis(q);
        Eigen::Vector3d x = angleAxis.angle() * angleAxis.axis();
        //std::cout << "angle-axis: " << x << std::endl;

        double delta = 1e-3;
        Eigen::Vector3d x_plus_dx(x.x()+delta, x.y(),       x.z());
        //std::cout << "angle-axis plus dx: " << x_plus_dx << std::endl;
        Eigen::Vector3d x_plus_dy(x.x(),       x.y()+delta, x.z());
        //std::cout << "angle-axis plus dy: " << x_plus_dy << std::endl;
        Eigen::Vector3d x_plus_dz(x.x(),       x.y(),       x.z()+delta);
        //std::cout << "angle-axis plus dz: " << x_plus_dz << std::endl;
        Eigen::Vector3d x_minus_dx(x.x()-delta, x.y(),        x.z());
        Eigen::Vector3d x_minus_dy(x.x(),       x.y()-delta,  x.z());
        Eigen::Vector3d x_minus_dz(x.x(),       x.y(),        x.z()-delta);

        //std::cout << "quaternion " << q.x() << ", " << q.y()  << ", " << q.z() << ", " << q.w() << std::endl;
        Eigen::Quaterniond q_plus_dx(Eigen::AngleAxisd(x_plus_dx.norm(), x_plus_dx/x_plus_dx.norm()));
        //std::cout << "quaternion plus dx: " << q_plus_dx.coeffs() << std::endl;
        Eigen::Quaterniond q_plus_dy(Eigen::AngleAxisd(x_plus_dy.norm(), x_plus_dy/x_plus_dy.norm()));
        //std::cout << "quaternion plus dy: " << q_plus_dy.coeffs() << std::endl;
        Eigen::Quaterniond q_plus_dz(Eigen::AngleAxisd(x_plus_dz.norm(), x_plus_dz/x_plus_dz.norm()));
        //std::cout << "quaternion plus dz: " << q_plus_dz.coeffs() << std::endl;
        Eigen::Quaterniond q_minus_dx(Eigen::AngleAxisd(x_minus_dx.norm(), x_minus_dx/x_minus_dx.norm()));
        Eigen::Quaterniond q_minus_dy(Eigen::AngleAxisd(x_minus_dy.norm(), x_minus_dy/x_minus_dy.norm()));
        Eigen::Quaterniond q_minus_dz(Eigen::AngleAxisd(x_minus_dz.norm(), x_minus_dz/x_minus_dz.norm()));

        Eigen::Vector2d f, f_plus_dx, f_plus_dy, f_plus_dz, f_minus_dx, f_minus_dy, f_minus_dz;
        projectQuaternion(eigenUnitQuaternionRotation[0], cameraPosition, sensor, track3D, f.data());
        //std::cout << "f(x): " << f << std::endl;
        projectQuaternion(q_plus_dx.coeffs().data(), cameraPosition, sensor, track3D, f_plus_dx.data());
        //std::cout << "f(x+dx): " << f_plus_dx << std::endl;
        projectQuaternion(q_plus_dy.coeffs().data(), cameraPosition, sensor, track3D, f_plus_dy.data());
        //std::cout << "f(x+dy): " << f_plus_dy << std::endl;
        projectQuaternion(q_plus_dz.coeffs().data(), cameraPosition, sensor, track3D, f_plus_dz.data());
        //std::cout << "f(x+dz): " << f_plus_dz << std::endl;
        projectQuaternion(q_minus_dx.coeffs().data(), cameraPosition, sensor, track3D, f_minus_dx.data());
        projectQuaternion(q_minus_dy.coeffs().data(), cameraPosition, sensor, track3D, f_minus_dy.data());
        projectQuaternion(q_minus_dz.coeffs().data(), cameraPosition, sensor, track3D, f_minus_dz.data());

        Eigen::Vector2d fdx_p = (f_plus_dx - f)/ delta;
        Eigen::Vector2d fdy_p = (f_plus_dy - f)/ delta;
        Eigen::Vector2d fdz_p = (f_plus_dz - f)/ delta;
        Eigen::Matrix<double, 2, 3> numerical_J_project_angleAxis_plus;
        numerical_J_project_angleAxis_plus << fdx_p(0), fdy_p(0), fdz_p(0),
                                              fdx_p(1), fdy_p(1), fdz_p(1);
        std::cout << "Num Jac plus: " << numerical_J_project_angleAxis_plus << std::endl;

        Eigen::Vector2d fdx_m = (f - f_minus_dx) / delta;
        Eigen::Vector2d fdy_m = (f - f_minus_dy) / delta;
        Eigen::Vector2d fdz_m = (f - f_minus_dz) / delta;
        Eigen::Matrix<double, 2, 3> numerical_J_project_angleAxis_minus;
        numerical_J_project_angleAxis_minus << fdx_m(0), fdy_m(0), fdz_m(0),
                                               fdx_m(1), fdy_m(1), fdz_m(1);
        std::cout << "Num Jac minus: " << numerical_J_project_angleAxis_minus << std::endl;

      }

      return true;
    }

  private:
    const Eigen::Vector3d& cameraPosition;
    const CameraSensor& sensor;
    const Eigen::Vector2d& mark;
    const Eigen::Vector3d& track3D;
  };
}
