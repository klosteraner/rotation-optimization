#pragma once

#include "types.h"
#include "geometry.hpp"

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace roto
{
/// Cost function for automatic derivative
/// using global angle-axis parametrization
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

/// cost function for automatic derivative
/// using local angle-axis parametrization (Singularity-free)
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

/// Cost function analytical derivative
/// using local angle-axis parametrization (Singularity-free)
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
      std::cout << "current det: " << q.norm() << std::endl;

      // Some calculations are done again: Ok for now.
      J_projectQuaternion_q(eigenUnitQuaternionRotation[0], cameraPosition, sensor, track3D, jacobians[0]);
    }

    return true;
  }

private:
  const Eigen::Vector3d& cameraPosition;
  const CameraSensor& sensor;
  const Eigen::Vector2d& mark;
  const Eigen::Vector3d& track3D;
};

/// cost function for automatic derivative
/// using local angle-axis parametrization (Singularity-free)
struct RotationMatrixReprojectionCostFunctorAuto
{
  RotationMatrixReprojectionCostFunctorAuto(const Eigen::Vector3d& _cameraPosition, const CameraSensor& _sensor, const Eigen::Vector2d& _mark, const Eigen::Vector3d& _track3D)
  : cameraPosition(_cameraPosition), sensor(_sensor), mark(_mark), track3D(_track3D){}

  template <typename T>
  bool operator()(const T* rotationMatrix, T* residuals) const
  {
    T projection[2];
    projectMatrix(rotationMatrix, cameraPosition, sensor, track3D, projection);
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

/// Cost function analytical derivative
/// using local angle-axis parametrization (Singularity-free)
///TODO: Probably buggy, use version below
struct RotationMatrixReprojectionCostFunctionAna : public ceres::SizedCostFunction<2, 9>
{
  RotationMatrixReprojectionCostFunctionAna(const Eigen::Vector3d& _cameraPosition, const CameraSensor& _sensor, const Eigen::Vector2d& _mark, const Eigen::Vector3d& _track3D)
  : cameraPosition(_cameraPosition), sensor(_sensor), mark(_mark), track3D(_track3D){}

  bool Evaluate(double const* const* rotationMatrix, double* residuals, double** jacobians) const
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

private:
  const Eigen::Vector3d& cameraPosition;
  const CameraSensor& sensor;
  const Eigen::Vector2d& mark;
  const Eigen::Vector3d& track3D;
};

/// Same(!) cost function analytical derivative
/// as RotationMatrixReprojectionCostFunctionAna but with less operations
/// using (singularity-free) local angle-axis parametrization
struct RotationMatrixReprojectionCostFunctionAna2 : public ceres::SizedCostFunction<2, 9>
{
  RotationMatrixReprojectionCostFunctionAna2(const Eigen::Vector3d& _cameraPosition, const CameraSensor& _sensor, const Eigen::Vector2d& _mark, const Eigen::Vector3d& _track3D)
  : cameraPosition(_cameraPosition),
    sensor(_sensor),
    mark(_mark),
    track3D(_track3D){}

  bool Evaluate(double const* const* rowMajorRotationMatrix, double* residuals, double** jacobians) const
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

private:
  const Eigen::Vector3d& cameraPosition;
  const CameraSensor& sensor;
  const Eigen::Vector2d& mark;
  const Eigen::Vector3d& track3D;
};

class RotationMatrixParameterization : public ceres::LocalParameterization
{
public:
  virtual ~RotationMatrixParameterization() {}

  // Parametrization at R
  // Note that this plus chains in the opposite order than Ceres
  // Parametrizations. It follows instead the convention used in
  // https://arxiv.org/pdf/1812.01537.pdf (Joan Sola)
  bool Plus(const double* R, const double* delta, double* R_plus_delta) const
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

  bool ComputeJacobian(const double* R, double* J_R_delta_at_zero) const
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

  int GlobalSize() const override {return 9;}
  int LocalSize() const override {return 3;}
};
} // namespace roto
