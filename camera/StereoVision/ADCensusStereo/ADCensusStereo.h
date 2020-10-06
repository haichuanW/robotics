
#include <opencv2/opencv.hpp>
#include "adcensus_types.h"
#include "cost_computor.h"
#include "cross_aggregator.h"
#include "multistep_refiner.h"
#include "scanline_optimizer.h"

class ADCensusStereo {
   public:
    ADCensusStereo();
    ~ADCensusStereo();

    bool Initialize(const int& width, const int& height, const ADCensusOption& option);

    // bool Match(const uint8* img_left, const uint8* img_right, float* disp_left);
    bool Match(const cv::Mat& img_left, const cv::Mat& img_right, cv::Mat& disp_left);

    bool Reset(const size_t& width, const size_t& height, const ADCensusOption& option);

   private:
    void ComputeCost();

    void CostAggregation();

    void ScanlineOptimize();

    void MultiStepRefine();

    void ComputeDisparity();

    void ComputeDisparityRight();

    void Release();

   private:
    ADCensusOption option_;

    int width_;
    int height_;

    const unsigned char* img_left_;
    const unsigned char* img_right_;

    CostComputor cost_computer_;
    CrossAggregator aggregator_;
    ScanlineOptimizer scan_line_;
    MultiStepRefiner refiner_;

    float* disp_left_;
    float* disp_right_;

    bool is_initialized_;
};
