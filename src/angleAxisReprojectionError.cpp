#include "angleAxisReprojectionError.hpp"

namespace roto
{
void addAngleAxisReprojectionError(ceres::Problem& problem, const MeasuredScene<AngleAxisRotation>& measurements, OptimizedScene<AngleAxisRotation>& parameters)
{
  for(const auto& track : measurements.tracks)
  {
    for(const auto& camIdAndMark : track.marks)
    {
      const auto& camera = measurements.cameras.at(camIdAndMark.first);

      ceres::CostFunction* f =
        new ceres::AutoDiffCostFunction<GlobalAngleAxisReprojectionCostFunctorAuto, 2, 3>(
          new GlobalAngleAxisReprojectionCostFunctorAuto(camera.pose.position,
                                                   measurements.cameraSensors.at(camera.sensorId),
                                                   camIdAndMark.second,
                                                   track.originalPosition));

      problem.AddResidualBlock(f, nullptr, parameters.rotation[camIdAndMark.first].data());
    }
  }
}
} // namespace roto
