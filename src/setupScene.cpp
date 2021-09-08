#include "setupScene.h"

#include "geometry.hpp"

#include "sophus/common.hpp"

#include <random>

namespace sote
{
  Eigen::Vector3d noise3d(double sigma)
  {
    std::random_device rd{};
    std::mt19937 gen(rd());

    std::normal_distribution<> normal(0., sigma);
    return Eigen::Vector3d(normal(gen), normal(gen), normal(gen));
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

  template<typename RotationType> RotationType lookingDown();
  template<> AngleAxisRotation lookingDown<AngleAxisRotation>()
  {
    return AngleAxisRotation::Zero();
  }
  template<> QuaternionRotation lookingDown<QuaternionRotation>()
  {
    return QuaternionRotation::Identity();
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

  template <typename RotationType>
  std::pair<MeasuredScene<RotationType>, OptimizedScene<RotationType>> setupTestSceneImpl()
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

    scene.cameraSensors.push_back(CameraSensor(40, 30, 20, 20, 15));

    //const RotationType lookingDown = lookingDown<RotationType>();

    scene.cameras.push_back(
      Camera<RotationType>{CameraPose<RotationType>{
        Eigen::Vector3d(0., 0., 2.),  // position
        lookingDown<RotationType>()},                 // rotation
        0});						              // sensor
    scene.cameras.push_back(
      Camera<RotationType>{CameraPose<RotationType>{
        Eigen::Vector3d(1., 0., 2.),  // position
        lookingDown<RotationType>()},                 // rotation
        0});							            // sensor
    scene.cameras.push_back(
      Camera<RotationType>{CameraPose<RotationType>{
        Eigen::Vector3d(0., 1., 2.),  // position
        lookingDown<RotationType>()},                 // rotation
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

  template<>
  std::pair<MeasuredScene<AngleAxisRotation>, OptimizedScene<AngleAxisRotation>> setupTestScene()
  {
    return setupTestSceneImpl<AngleAxisRotation>();
  }

  template<>
  std::pair<MeasuredScene<QuaternionRotation>, OptimizedScene<QuaternionRotation>> setupTestScene()
  {
    return setupTestSceneImpl<QuaternionRotation>();
  }
}
