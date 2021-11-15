#pragma once

#include "types.h"

#include <vector>

namespace roto
{
// Setups & runs optimization on provided parameters
// return processing time (excluding problem setup and report printing) 
template<typename RotationType>
double optimize(const MeasuredScene<RotationType>& measurements, OptimizedScene<RotationType>& parameters);
}
