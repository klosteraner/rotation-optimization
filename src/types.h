#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <vector>
#include <map>

namespace roto
{
struct CameraSensor
{
	int w;
	int h;

	double f;
	double cx;
	double cy;
};

using AngleAxisRotation = Eigen::Vector3d;
using QuaternionRotation = Eigen::Quaterniond;
// RowMajor preferredm since it follows more intuitive indexing when unrolling:
// (r11, r12, r13,
//	r21, r22, r23,	-> (r11, r12, r13, r21, ...)
//	r31, r32, r33)
// TODO: Major whether there is performance difference using default ColMajor
using MatrixRotation = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

template<typename RotationType>
struct CameraPose
{
  Eigen::Vector3d position;
  RotationType rotation;
};

template<typename RotationType>
struct Camera
{
  CameraPose<RotationType> pose;
  std::size_t sensorId;
};

struct Track
{
  Eigen::Vector3d originalPosition;
  std::map<std::size_t, Eigen::Vector2d> marks;
};

// Container for measurements
template<typename RotationType>
struct MeasuredScene
{
  std::vector<CameraSensor> cameraSensors;
  std::vector<Camera<RotationType>> cameras;
  std::vector<Track> tracks;
};

template<typename RotationType>
struct OptimizedScene
{
  std::vector<RotationType> rotation;
};
} // namespace roto
