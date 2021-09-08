#pragma once

#include "types.h"

namespace sote
{
    Eigen::Vector3d noise3d(double sigma = 1);

    template<typename RotationType>
    std::pair<MeasuredScene<RotationType>, OptimizedScene<RotationType>> setupTestScene();
}
