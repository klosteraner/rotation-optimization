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
