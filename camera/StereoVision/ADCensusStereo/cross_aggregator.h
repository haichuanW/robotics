#ifndef AD_CENSUS_CROSS_AGGREGATOR_H_
#define AD_CENSUS_CROSS_AGGREGATOR_H_

#include <algorithm>
#include "adcensus_types.h"

struct CrossArm {
    uint8 left, right, top, bottom;
    CrossArm() : left(0), right(0), top(0), bottom(0) {}
};

#define MAX_ARM_LENGTH 255

class CrossAggregator {
   public:
    CrossAggregator();
    ~CrossAggregator();

    bool Initialize(const int& width, const int& height, const int& min_disparity,
                    const int& max_disparity);

    void SetData(const uint8* img_left, const uint8* img_right, const float* cost_init);

    void SetParams(const int& cross_L1, const int& cross_L2, const int& cross_t1,
                   const int& cross_t2);

    void Aggregate(const int& num_iters);

    CrossArm* get_arms_ptr();

    float* get_cost_ptr();

   private:
    void BuildArms();
    void FindHorizontalArm(const int& x, const int& y, uint8& left, uint8& right) const;
    void FindVerticalArm(const int& x, const int& y, uint8& top, uint8& bottom) const;
    void ComputeSupPixelCount();
    void AggregateInArms(const int& disparity, const bool& horizontal_first);

    inline int ColorDist(const ADColor& c1, const ADColor& c2) const {
        return std::max(abs(c1.r - c2.r), std::max(abs(c1.g - c2.g), abs(c1.b - c2.b)));
    }

   private:
    int width_;
    int height_;

    vector<CrossArm> vec_cross_arms_;

    const uint8* img_left_;
    const uint8* img_right_;

    const float* cost_init_;
    vector<float> cost_aggr_;

    vector<float> vec_cost_tmp_[2];
    vector<uint16_t> vec_sup_count_[2];
    vector<uint16_t> vec_sup_count_tmp_;

    int cross_L1_;
    int cross_L2_;
    int cross_t1_;
    int cross_t2_;
    int min_disparity_;
    int max_disparity_;

    bool is_initialized_;
};
#endif
