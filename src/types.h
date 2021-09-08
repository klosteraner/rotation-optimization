#pragma once

#include <Eigen/Geometry>

#include <memory>
#include <vector>
#include <map>

namespace sote
{
struct CameraSensor
{
  CameraSensor(int _w, int _h, double _f, double _cx, double _cy)
  : w(_w),
    h(_h),
    f(_f),
    cx(_cx),
    cy(_cy){}

  CameraSensor() : CameraSensor{2, 2, 2. ,1. ,1.} {}

	int w;
	int h;

	double f;
	double cx;
	double cy;
};

using Rotation = Eigen::Vector3d; // angle axis

struct CameraPose
{
  Eigen::Vector3d position;
  Rotation rotation;
};

struct Camera
{
  CameraPose pose;
  std::size_t sensorId;
};

struct Track
{
  Eigen::Vector3d originalPosition;
  std::map<std::size_t, Eigen::Vector2d> marks;
};

// Container for measurements
struct MeasuredScene
{
  std::vector<CameraSensor> cameraSensors;
  std::vector<Camera> cameras;
  std::vector<Track> tracks;
};

struct OptimizedScene
{
  std::vector<Rotation> rotation;
};
}
