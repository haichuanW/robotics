#ifndef DISPARITYREFINEMENT_H
#define DISPARITYREFINEMENT_H

#include "adcensuscv.h"
#include "common.h"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class DisparityRefinement {
public:
  DisparityRefinement(uint dispTolerance, int dMin, int dMax,
                      uint votingThreshold, float votingRatioThreshold,
                      uint maxSearchDepth, uint blurKernelSize,
                      uint cannyThreshold1, uint cannyThreshold2,
                      uint cannyKernelSize);
  Mat outlierElimination(const Mat &leftDisp, const Mat &rightDisp);
  void regionVoting(Mat &disparity, const vector<Mat> &upLimits,
                    const vector<Mat> &downLimits,
                    const vector<Mat> &leftLimits,
                    const vector<Mat> &rightLimits, bool horizontalFirst);
  void properInterpolation(Mat &disparity, const Mat &leftImage);
  void discontinuityAdjustment(Mat &disparity,
                               const vector<vector<Mat>> &costs);
  Mat subpixelEnhancement(Mat &disparity, const vector<vector<Mat>> &costs);

  static const int DISP_OCCLUSION;
  static const int DISP_MISMATCH;

private:
  int colorDiff(const Vec3b &p1, const Vec3b &p2);
  Mat convertDisp2Gray(const Mat &disparity);

  int occlusionValue;
  int mismatchValue;
  uint dispTolerance;
  int dMin;
  int dMax;
  uint votingThreshold;
  float votingRatioThreshold;
  uint maxSearchDepth;
  uint blurKernelSize;
  uint cannyThreshold1;
  uint cannyThreshold2;
  uint cannyKernelSize;
};

#endif // DISPARITYREFINEMENT_H
