#pragma once

#include "types.h"

#include <vector>

namespace sote
{
  template<typename RotationType>
  void optimize(const MeasuredScene<RotationType>& measurements, OptimizedScene<RotationType>& parameters);
}
