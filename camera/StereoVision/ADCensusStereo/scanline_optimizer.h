#ifndef AD_CENSUS_SCANLNE_OPTIMIZER_H_
#define AD_CENSUS_SCANLNE_OPTIMIZER_H_

#include <algorithm>

#include "adcensus_types.h"

class ScanlineOptimizer {
   public:
    ScanlineOptimizer();
    ~ScanlineOptimizer();

    void SetData(const uint8* img_left, const uint8* img_right, float* cost_init, float* cost_aggr);

    void SetParam(const int& width, const int& height, const int& min_disparity,
                  const int& max_disparity, const float& p1, const float& p2, const int& tso);

    void Optimize();

   private:
    void CostAggregateLeftRight(const float* cost_so_src, float* cost_so_dst,
                                bool is_forward = true);

    void CostAggregateUpDown(const float* cost_so_src, float* cost_so_dst, bool is_forward = true);

    inline int ColorDist(const ADColor& c1, const ADColor& c2) {
        return std::max(abs(c1.r - c2.r), std::max(abs(c1.g - c2.g), abs(c1.b - c2.b)));
    }

   private:
    int width_;
    int height_;

    const uint8* img_left_;
    const uint8* img_right_;

    float* cost_init_;
    float* cost_aggr_;

    int min_disparity_;
    int max_disparity_;
    float so_p1_;
    float so_p2_;
    int so_tso_;
};
#endif
