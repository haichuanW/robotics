#ifndef STEREOPROCESSOR_H
#define STEREOPROCESSOR_H
#include "adcensuscv.h"
#include "aggregation.h"
#include "common.h"
#include "disparityrefinement.h"
#include "scanlineoptimization.h"
#include <omp.h>

using namespace std;

class StereoProcessor {
public:
  StereoProcessor(uint dMin, uint dMax, Mat leftImage, Mat rightImage,
                  Size censusWin, float defaultBorderCost, float lambdaAD,
                  float lambdaCensus, uint aggregatingIterations,
                  uint colorThreshold1, uint colorThreshold2, uint maxLength1,
                  uint maxLength2, uint colorDifference, float pi1, float pi2,
                  uint dispTolerance, uint votingThreshold,
                  float votingRatioThreshold, uint maxSearchDepth,
                  uint blurKernelSize, uint cannyThreshold1,
                  uint cannyThreshold2, uint cannyKernelSize);

  StereoProcessor(uint dMin, uint dMax, Size censusWin, float defaultBorderCost,
                  float lambdaAD, float lambdaCensus,
                  uint aggregatingIterations, uint colorThreshold1,
                  uint colorThreshold2, uint maxLength1, uint maxLength2,
                  uint colorDifference, float pi1, float pi2,
                  uint dispTolerance, uint votingThreshold,
                  float votingRatioThreshold, uint maxSearchDepth,
                  uint blurKernelSize, uint cannyThreshold1,
                  uint cannyThreshold2, uint cannyKernelSize);

  ~StereoProcessor();
  bool init();
  bool compute();

  bool compute(const cv::Mat &img_left, const cv::Mat &img_right);
  Mat getDisparity() const;

private:
  int dMin;
  int dMax;
  Mat images[2];
  Size censusWin;
  float defaultBorderCost;
  float lambdaAD;
  float lambdaCensus;
  uint aggregatingIterations;
  uint colorThreshold1;
  uint colorThreshold2;
  uint maxLength1;
  uint maxLength2;
  uint colorDifference;
  float pi1;
  float pi2;
  uint dispTolerance;
  uint votingThreshold;
  float votingRatioThreshold;
  uint maxSearchDepth;
  uint blurKernelSize;
  uint cannyThreshold1;
  uint cannyThreshold2;
  uint cannyKernelSize;
  bool validParams, dispComputed;

  vector<vector<Mat>> costMaps;
  Size imgSize;
  ADCensusCV *adCensus;
  Aggregation *aggregation;
  Mat disparityMap, floatDisparityMap;
  DisparityRefinement *dispRef;

  void costInitialization();
  void costAggregation();
  void scanlineOptimization();
  void outlierElimination();
  void regionVoting();
  void properInterpolation();
  void discontinuityAdjustment();
  void subpixelEnhancement();

  Mat cost2disparity(int imageNo);
};

#endif // STEREOPROCESSOR_H
