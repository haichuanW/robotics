
#pragma once
#include <algorithm>
#include "adcensus_types.h"

namespace adcensus_util {

void census_transform_9x7(const uint8* source, vector<size_t>& census, const int& width,
                          const int& height);
// Hamming distance
uint8 Hamming64(const size_t& x, const size_t& y);

void MedianFilter(const float* in, float* out, const int& width, const int& height,
                  const int wnd_size);
}  // namespace adcensus_util