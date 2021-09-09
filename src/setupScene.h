#pragma once

#include "types.h"

namespace sote
{
template<typename RotationType>
std::pair<MeasuredScene<RotationType>, OptimizedScene<RotationType>> setupSmallTestScene();

template<typename RotationType>
std::pair<MeasuredScene<RotationType>, OptimizedScene<RotationType>> setupBigTestScene();
}
