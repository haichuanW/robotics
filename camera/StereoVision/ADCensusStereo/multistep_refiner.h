#ifndef AD_CENSUS_MULTISTEP_REFINER_H_
#define AD_CENSUS_MULTISTEP_REFINER_H_

#include "adcensus_types.h"
#include "cross_aggregator.h"

class MultiStepRefiner {
   public:
    MultiStepRefiner();
    ~MultiStepRefiner();

    bool Initialize(const int& width, const int& height);

    void SetData(const uint8* img_left, float* cost, const CrossArm* cross_arms, float* disp_left,
                 float* disp_right);

    void SetParam(const int& min_disparity, const int& max_disparity, const int& irv_ts,
                  const float& irv_th, const float& lrcheck_thres, const bool& do_lr_check,
                  const bool& do_region_voting, const bool& do_interpolating,
                  const bool& do_discontinuity_adjustment);

    void Refine();

   private:
    void OutlierDetection();
    void IterativeRegionVoting();
    void ProperInterpolation();
    void DepthDiscontinuityAdjustment();

    static void EdgeDetect(uint8* edge_mask, const float* disp_ptr, const int& width,
                           const int& height, const float threshold);

   private:
    int width_;
    int height_;

    const uint8* img_left_;

    float* cost_;
    const CrossArm* cross_arms_;

    float* disp_left_;
    float* disp_right_;

    vector<uint8> vec_edge_left_;

    int min_disparity_;
    int max_disparity_;

    int irv_ts_;
    float irv_th_;

    float lrcheck_thres_;

    bool do_lr_check_;
    bool do_region_voting_;
    bool do_interpolating_;
    bool do_discontinuity_adjustment_;

    vector<pair<int, int>> occlusions_;
    vector<pair<int, int>> mismatches_;
};
#endif
