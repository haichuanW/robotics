#ifndef AD_CENSUS_COST_COMPUTOR_H_
#define AD_CENSUS_COST_COMPUTOR_H_

#include "adcensus_types.h"

class CostComputor {
   public:
    CostComputor();
    ~CostComputor();

    bool Initialize(const int& width, const int& height, const int& min_disparity,
                    const int& max_disparity);

    void SetData(const uint8* img_left, const uint8* img_right);

    void SetParams(const int& lambda_ad, const int& lambda_census);

    void Compute();

    float* get_cost_ptr();

   private:
    void ComputeGray();

    void CensusTransform();

    void ComputeCost();

   private:
    int width_;
    int height_;

    const uint8* img_left_;
    const uint8* img_right_;

    vector<uint8> gray_left_;
    vector<uint8> gray_right_;

    vector<size_t> census_left_;
    vector<size_t> census_right_;

    vector<float> cost_init_;

    int lambda_ad_;
    int lambda_census_;

    int min_disparity_;
    int max_disparity_;

    bool is_initialized_;
};
#endif