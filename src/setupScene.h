#pragma once

#include "types.h"

namespace sote
{
    Eigen::Vector3d noise3d(double sigma = 1);

    std::pair<MeasuredScene, OptimizedScene> setupTestScene();
}
