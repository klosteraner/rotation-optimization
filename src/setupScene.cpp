#include "setupScene.h"

#include "geometry.hpp"

#include <random>

namespace roto
{
namespace
{
Eigen::Vector3d noise3d(double sigma)
{
  std::random_device rd{};
  std::mt19937 gen(rd());

  std::normal_distribution<> normal(0., sigma);
  return Eigen::Vector3d(normal(gen), normal(gen), normal(gen));
}

Eigen::Vector3d random3d(const Eigen::Vector3d& center, double sigma_x, double sigma_y, double sigma_z)
{
  std::random_device rd{};
  std::mt19937 gen(rd());

  std::normal_distribution<> normal_x(0., sigma_x);
  std::normal_distribution<> normal_y(0., sigma_y);
  std::normal_distribution<> normal_z(0., sigma_z);
  return Eigen::Vector3d(normal_x(gen), normal_y(gen), normal_z(gen));
}

template<typename RotationType> RotationType applyNoise(const RotationType& rotation, double sigma);
template<> AngleAxisRotation applyNoise(const AngleAxisRotation& rotation, double sigma)
{
  return rotation + noise3d(sigma);
}
template<> QuaternionRotation applyNoise(const QuaternionRotation& rotation, double sigma)
{
  const Eigen::Vector3d noise = noise3d(sigma);
  const double noiseAngle = noise.norm();
  const Eigen::Vector3d noiseAxis = noise / noiseAngle;

  return rotation * Eigen::Quaterniond(Eigen::AngleAxisd(noiseAngle, noiseAxis));
}
template<> MatrixRotation applyNoise(const MatrixRotation& rotation, double sigma)
{
  const Eigen::Vector3d noise = noise3d(sigma);
  const double noiseAngle = noise.norm();
  const Eigen::Vector3d noiseAxis = noise / noiseAngle;

  return rotation * Eigen::Matrix3d(Eigen::AngleAxisd(noiseAngle, noiseAxis));
}


template<typename RotationType> RotationType lookingDown();
template<> AngleAxisRotation lookingDown<AngleAxisRotation>()
{
  return AngleAxisRotation::Zero();
}
template<> QuaternionRotation lookingDown<QuaternionRotation>()
{
  return QuaternionRotation::Identity();
}
template<> MatrixRotation lookingDown<MatrixRotation>()
{
  return MatrixRotation::Identity();
}

template<typename RotationType> void print(const RotationType& rotation);
template<> void print(const AngleAxisRotation& rotation)
{
  std::cout << "initial angle_axis: " << rotation << std::endl;
}
template<> void print(const QuaternionRotation& rotation)
{
  std::cout << "initial quaternion: " << rotation.coeffs() << std::endl;
}
template<> void print(const MatrixRotation& rotation)
{
  std::cout << "initial matrix: " << rotation << std::endl;
}

template <typename RotationType>
std::pair<MeasuredScene<RotationType>, OptimizedScene<RotationType>> setupSmallTestSceneImpl()
{
  /*
   * Scene:
   *            *cam2
   *      *cam0 |     *cam1
   *      |     v     |
   *      v           v
   *            *track2
   *      *track0     *track1
   *  z  y
   *  |/_x
   */

  MeasuredScene<RotationType> scene;

  scene.cameraSensors.push_back(CameraSensor{40, 30, 20, 20, 15});

  scene.cameras.push_back(
    Camera<RotationType>{CameraPose<RotationType>{
      Eigen::Vector3d(0., 0., 2.),  // position
      lookingDown<RotationType>()}, // rotation
      0});						              // sensor
  scene.cameras.push_back(
    Camera<RotationType>{CameraPose<RotationType>{
      Eigen::Vector3d(1., 0., 2.),  // position
      lookingDown<RotationType>()}, // rotation
      0});							            // sensor
  scene.cameras.push_back(
    Camera<RotationType>{CameraPose<RotationType>{
      Eigen::Vector3d(0., 1., 2.),  // position
      lookingDown<RotationType>()}, // rotation
      0});							            // sensor

  // Tracks
  Track mtp1;
  mtp1.originalPosition = Eigen::Vector3d(0., 0., 0.);
  mtp1.marks.emplace(0, project(scene.cameras[0].pose, scene.cameraSensors.at(0), mtp1.originalPosition));
  mtp1.marks.emplace(1, project(scene.cameras[1].pose, scene.cameraSensors.at(0), mtp1.originalPosition));
  mtp1.marks.emplace(2, project(scene.cameras[2].pose, scene.cameraSensors.at(0), mtp1.originalPosition));
  scene.tracks.push_back(mtp1);

  Track mtp2;
  mtp2.originalPosition = Eigen::Vector3d(1., 0., 0.);
  mtp2.marks.emplace(0, project(scene.cameras[0].pose, scene.cameraSensors.at(0), mtp2.originalPosition));
  mtp2.marks.emplace(1, project(scene.cameras[1].pose, scene.cameraSensors.at(0), mtp2.originalPosition));
  mtp2.marks.emplace(2, project(scene.cameras[2].pose, scene.cameraSensors.at(0), mtp2.originalPosition));
  scene.tracks.push_back(mtp2);

  Track mtp3;
  mtp3.originalPosition = Eigen::Vector3d(0., 1., 0.);
  mtp3.marks.emplace(0, project(scene.cameras[0].pose, scene.cameraSensors.at(0), mtp3.originalPosition));
  mtp3.marks.emplace(1, project(scene.cameras[1].pose, scene.cameraSensors.at(0), mtp3.originalPosition));
  mtp3.marks.emplace(2, project(scene.cameras[2].pose, scene.cameraSensors.at(0), mtp3.originalPosition));
  scene.tracks.push_back(mtp3);

  OptimizedScene<RotationType> parameters;

  parameters.rotation.reserve(scene.cameras.size());
  for(int i = 0; i < scene.cameras.size(); i++)
  {
    parameters.rotation.push_back(applyNoise(scene.cameras.at(i).pose.rotation, 0.1));
    print(parameters.rotation.at(i));
  }

  return std::make_pair(scene, parameters);
}

template <typename RotationType>
std::pair<MeasuredScene<RotationType>, OptimizedScene<RotationType>>
setupBigTestSceneImpl(const std::pair<std::size_t, std::size_t> cameraGridDimensions, std::size_t numberOfPoints)
{
  /*
   * Scene:
   *                  *cam((m-1)*n)   ...   *cam(m*n-1)
   *              ... |                  ...|
   *            *cam2 v              ...    v
   *      *cam0 |     *cam1  .... *cam(n)
   *      |     v     |           |
   *      v           v           v
   *
   *         *************************
   *    *********** Pointcloud ****************
   *         *************************
   *  z  y
   *  |/_x
   */

  MeasuredScene<RotationType> scene;
  scene.cameraSensors.push_back(CameraSensor{40, 30, 20, 20, 15});
  const auto& sensor = scene.cameraSensors.at(0);

  for(std::size_t i = 0; i < cameraGridDimensions.first; i++)
  {
    for(std::size_t j = 0; j < cameraGridDimensions.second; j++)
    {
      scene.cameras.push_back(
        Camera<RotationType>{CameraPose<RotationType>{
          Eigen::Vector3d(i*10., j*10., 50.) + noise3d(1.),  // position
          applyNoise(lookingDown<RotationType>(), 0.1)},     // rotation
          0});						                                   // sensor
    }
  }

  const Eigen::Vector3d pointCenter(5*cameraGridDimensions.first, 5*cameraGridDimensions.second, 0.);
  double numberOfReprojections = 0.;
  std::size_t maxLength = 0u;
  for(std::size_t k = 0; k < numberOfPoints; k++)
  {
    Track track;
    track.originalPosition = random3d(pointCenter, 8*cameraGridDimensions.first,  8*cameraGridDimensions.second, 5.);

    for(std::size_t cameraIndex = 0; cameraIndex < scene.cameras.size(); cameraIndex++)
    {
      const Eigen::Vector2d projection = project(scene.cameras.at(cameraIndex).pose, sensor, track.originalPosition);
      if(0 <= projection(0) && projection(0) < sensor.w &&
         0 <= projection(1) && projection(0) < sensor.h)
      {
        track.marks.emplace(cameraIndex, projection);
      }
    }
    if(track.marks.size() >= 3)
    {
      scene.tracks.push_back(track);
      numberOfReprojections += track.marks.size();
      if(maxLength < track.marks.size())
      {
        maxLength = track.marks.size();
      }
    }
  }
  std::cout << "Setup expected scene with" << std::endl;
  std::cout << "#Cameras: " << scene.cameras.size() << std::endl;
  std::cout << "#Tracks: " << scene.tracks.size() << std::endl;
  std::cout << "#reprojections: " << numberOfReprojections <<std::endl;
  std::cout << "average length: " << numberOfReprojections / scene.tracks.size() << std::endl;
  std::cout << "max length: " << maxLength << std::endl;

  OptimizedScene<RotationType> parameters;

  parameters.rotation.reserve(scene.cameras.size());
  for(int i = 0; i < scene.cameras.size(); i++)
  {
    parameters.rotation.push_back(applyNoise(scene.cameras.at(i).pose.rotation, 0.1));
  }

  return std::make_pair(scene, parameters);
}
} // namespace

template<>
std::pair<MeasuredScene<AngleAxisRotation>, OptimizedScene<AngleAxisRotation>> setupSmallTestScene()
{
  return setupSmallTestSceneImpl<AngleAxisRotation>();
}

template<>
std::pair<MeasuredScene<QuaternionRotation>, OptimizedScene<QuaternionRotation>> setupSmallTestScene()
{
  return setupSmallTestSceneImpl<QuaternionRotation>();
}

template<>
std::pair<MeasuredScene<MatrixRotation>, OptimizedScene<MatrixRotation>> setupSmallTestScene()
{
  return setupSmallTestSceneImpl<MatrixRotation>();
}

template<>
std::pair<MeasuredScene<AngleAxisRotation>, OptimizedScene<AngleAxisRotation>> setupBigTestScene()
{
  return setupBigTestSceneImpl<AngleAxisRotation>(std::make_pair(10,10), 1000);
}

template<>
std::pair<MeasuredScene<QuaternionRotation>, OptimizedScene<QuaternionRotation>> setupBigTestScene()
{
  return setupBigTestSceneImpl<QuaternionRotation>(std::make_pair(10,10), 1000);
}

template<>
std::pair<MeasuredScene<MatrixRotation>, OptimizedScene<MatrixRotation>> setupBigTestScene()
{
  return setupBigTestSceneImpl<MatrixRotation>(std::make_pair(10,10), 1000);
}
} // namespace roto
