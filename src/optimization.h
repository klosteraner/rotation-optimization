#pragma once

#include "types.h"

#include <vector>

namespace roto
{
// Setups & runs optimization on provided parameters
template<typename RotationType>
void optimize(const MeasuredScene<RotationType>& measurements, OptimizedScene<RotationType>& parameters);
}
