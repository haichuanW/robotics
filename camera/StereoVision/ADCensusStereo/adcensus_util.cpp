#include "adcensus_util.h"
#include <cassert>

void adcensus_util::census_transform_9x7(const uint8 *source, vector<size_t> &census,
                                         const int &width, const int &height) {
    if (source == nullptr || census.empty() || width <= 9 || height <= 7) {
        return;
    }

    for (int i = 4; i < height - 4; i++) {
        for (int j = 3; j < width - 3; j++) {
            const uint8 gray_center = source[i * width + j];

            size_t census_val = 0u;
            for (int r = -4; r <= 4; r++) {
                for (int c = -3; c <= 3; c++) {
                    census_val <<= 1;
                    const uint8 gray = source[(i + r) * width + j + c];
                    if (gray < gray_center) {
                        census_val += 1;
                    }
                }
            }

            census[i * width + j] = census_val;
        }
    }
}

uint8 adcensus_util::Hamming64(const size_t &x, const size_t &y) {
    size_t dist = 0, val = x ^ y;

    // Count the number of set bits
    while (val) {
        ++dist;
        val &= val - 1;
    }

    return static_cast<uint8>(dist);
}

void adcensus_util::MedianFilter(const float *in, float *out, const int &width, const int &height,
                                 const int wnd_size) {
    const int radius = wnd_size / 2;
    const int size = wnd_size * wnd_size;

    std::vector<float> wnd_data;
    wnd_data.reserve(size);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            wnd_data.clear();
            for (int r = -radius; r <= radius; r++) {
                for (int c = -radius; c <= radius; c++) {
                    const int row = y + r;
                    const int col = x + c;
                    if (row >= 0 && row < height && col >= 0 && col < width) {
                        wnd_data.push_back(in[row * width + col]);
                    }
                }
            }
            std::sort(wnd_data.begin(), wnd_data.end());
            if (!wnd_data.empty()) {
                out[y * width + x] = wnd_data[wnd_data.size() / 2];
            }
        }
    }
}