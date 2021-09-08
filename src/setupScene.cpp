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

  std::pair<MeasuredScene, OptimizedScene> setupTestSceneAngleAxis()
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

    MeasuredScene scene;

    scene.cameraSensors.push_back(CameraSensor(40, 30, 20, 20, 15));

    const auto lookingDown =
      Eigen::Vector3d(0., 0., 0.);
    scene.cameras.push_back(
      Camera{CameraPose{Eigen::Vector3d(0., 0., 2.),  // position
                        lookingDown},                 // rotation
             0});						  // sensor
    scene.cameras.push_back(
      Camera{CameraPose{Eigen::Vector3d(1., 0., 2.),  // position
                        lookingDown},                 // rotation
             0});							// sensor
    scene.cameras.push_back(
      Camera{CameraPose{Eigen::Vector3d(0., 1., 2.),  // position
                        lookingDown},                 // rotation
             0});							// sensor

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

    OptimizedScene parameters;

    parameters.rotation.reserve(scene.cameras.size());
    for(int i = 0; i < scene.cameras.size(); i++)
    {
      parameters.rotation.push_back(scene.cameras.at(i).pose.rotation + sote::noise3d(0.01));
      std::cout << "initial angle_axis: " << parameters.rotation.at(i) << std::endl;
    }

    return std::make_pair(scene, parameters);
  }

  std::pair<MeasuredScene, OptimizedScene> setupTestScene()
  {
    return setupTestSceneAngleAxis();
  }
}
