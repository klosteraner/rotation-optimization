#pragma once

#include "geometry.hpp"

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace roto
{
  template<typename T>
  void projectMatrix(const T rowMajorMatrixRotation[9], const Eigen::Vector3d& cameraPosition,
                     const CameraSensor& sensor, const Eigen::Vector3d& worldCoordinates, T projection[2])
  {
    Eigen::Map<const Eigen::Matrix<T, 3, 3, Eigen::RowMajor>> R(rowMajorMatrixRotation);
    const Eigen::Matrix<T, 3, 1> p(T(worldCoordinates[0] - cameraPosition[0]),
                                   T(worldCoordinates[1] - cameraPosition[1]),
                                   T(worldCoordinates[2] - cameraPosition[2]));
    const Eigen::Matrix<T, 3, 1> p_camera = R * p;

    project(p_camera.data(), sensor, projection);
  }

// Analytic derivative of the projectMatrix function
// Since its already the derivative, there is no need to support jets
void J_projectMatrix_R(const double matrixRotation[9], const Eigen::Vector3d& cameraPosition,
                       const CameraSensor& sensor, const Eigen::Vector3d& worldCoordinates, double jacobian[18]);

/// cost function for automatic derivative
/// using local angle-axis parametrization (Singularity-free)
struct RotationMatrixReprojectionCostFunctorAuto
{
  RotationMatrixReprojectionCostFunctorAuto(const Eigen::Vector3d& _cameraPosition, const CameraSensor& _sensor,
                                            const Eigen::Vector2d& _mark, const Eigen::Vector3d& _track3D)
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
  RotationMatrixReprojectionCostFunctionAna(const Eigen::Vector3d& _cameraPosition, const CameraSensor& _sensor,
                                            const Eigen::Vector2d& _mark, const Eigen::Vector3d& _track3D)
  : cameraPosition(_cameraPosition), sensor(_sensor), mark(_mark), track3D(_track3D){}

  bool Evaluate(double const* const* rotationMatrix, double* residuals, double** jacobians) const;

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
  RotationMatrixReprojectionCostFunctionAna2(const Eigen::Vector3d& _cameraPosition, const CameraSensor& _sensor,
                                             const Eigen::Vector2d& _mark, const Eigen::Vector3d& _track3D)
  : cameraPosition(_cameraPosition),
    sensor(_sensor),
    mark(_mark),
    track3D(_track3D){}

  bool Evaluate(double const* const* rowMajorRotationMatrix, double* residuals, double** jacobians) const;

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
  bool Plus(const double* R, const double* delta, double* R_plus_delta) const;

  bool ComputeJacobian(const double* R, double* J_R_delta_at_zero) const;

  int GlobalSize() const override {return 9;}
  int LocalSize() const override {return 3;}
};

void addMatrixReprojectionError(ceres::Problem& problem, const MeasuredScene<MatrixRotation>& measurements,
                                OptimizedScene<MatrixRotation>& parameters, bool useAnalyticDerivative = false);
} // namespace roto
