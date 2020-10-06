#include "stereoprocessor.h"
#include "common.h"
#include <chrono>
#include <limits>

// #define DEBUG

StereoProcessor::StereoProcessor(
    uint dMin, uint dMax, Mat leftImage, Mat rightImage, Size censusWin,
    float defaultBorderCost, float lambdaAD, float lambdaCensus,
    uint aggregatingIterations, uint colorThreshold1, uint colorThreshold2,
    uint maxLength1, uint maxLength2, uint colorDifference, float pi1,
    float pi2, uint dispTolerance, uint votingThreshold,
    float votingRatioThreshold, uint maxSearchDepth, uint blurKernelSize,
    uint cannyThreshold1, uint cannyThreshold2, uint cannyKernelSize) {
  this->dMin = dMin;
  this->dMax = dMax;
  this->images[0] = leftImage;
  this->images[1] = rightImage;
  this->censusWin = censusWin;
  this->defaultBorderCost = defaultBorderCost;
  this->lambdaAD = lambdaAD;
  this->lambdaCensus = lambdaCensus;
  this->aggregatingIterations = aggregatingIterations;
  this->colorThreshold1 = colorThreshold1;
  this->colorThreshold2 = colorThreshold2;
  this->maxLength1 = maxLength1;
  this->maxLength2 = maxLength2;
  this->colorDifference = colorDifference;
  this->pi1 = pi1;
  this->pi2 = pi2;
  this->dispTolerance = dispTolerance;
  this->votingThreshold = votingThreshold;
  this->votingRatioThreshold = votingRatioThreshold;
  this->maxSearchDepth = maxSearchDepth;
  this->blurKernelSize = blurKernelSize;
  this->cannyThreshold1 = cannyThreshold1;
  this->cannyThreshold2 = cannyThreshold2;
  this->cannyKernelSize = cannyKernelSize;
  this->validParams = false;
  this->dispComputed = false;
}

StereoProcessor::StereoProcessor(
    uint dMin, uint dMax, Size censusWin, float defaultBorderCost,
    float lambdaAD, float lambdaCensus, uint aggregatingIterations,
    uint colorThreshold1, uint colorThreshold2, uint maxLength1,
    uint maxLength2, uint colorDifference, float pi1, float pi2,
    uint dispTolerance, uint votingThreshold, float votingRatioThreshold,
    uint maxSearchDepth, uint blurKernelSize, uint cannyThreshold1,
    uint cannyThreshold2, uint cannyKernelSize) {
  this->dMin = dMin;
  this->dMax = dMax;
  this->censusWin = censusWin;
  this->defaultBorderCost = defaultBorderCost;
  this->lambdaAD = lambdaAD;
  this->lambdaCensus = lambdaCensus;
  this->aggregatingIterations = aggregatingIterations;
  this->colorThreshold1 = colorThreshold1;
  this->colorThreshold2 = colorThreshold2;
  this->maxLength1 = maxLength1;
  this->maxLength2 = maxLength2;
  this->colorDifference = colorDifference;
  this->pi1 = pi1;
  this->pi2 = pi2;
  this->dispTolerance = dispTolerance;
  this->votingThreshold = votingThreshold;
  this->votingRatioThreshold = votingRatioThreshold;
  this->maxSearchDepth = maxSearchDepth;
  this->blurKernelSize = blurKernelSize;
  this->cannyThreshold1 = cannyThreshold1;
  this->cannyThreshold2 = cannyThreshold2;
  this->cannyKernelSize = cannyKernelSize;
  this->validParams = false;
  this->dispComputed = false;
}

StereoProcessor::~StereoProcessor() {
  delete adCensus;
  delete aggregation;
  delete dispRef;
}

bool StereoProcessor::init() {
  bool valid = true;

  this->imgSize = images[0].size();
  costMaps.resize(2);
  for (size_t i = 0; i < 2; i++) {
    costMaps[i].resize(abs(dMax - dMin) + 1);
    for (size_t j = 0; j < costMaps[i].size(); j++) {
      costMaps[i][j].create(imgSize, COST_MAP_TYPE);
    }
  }

  adCensus =
      new ADCensusCV(images[0], images[1], censusWin, lambdaAD, lambdaCensus);
  aggregation = new Aggregation(images[0], images[1], colorThreshold1,
                                colorThreshold2, maxLength1, maxLength2);
  dispRef = new DisparityRefinement(dispTolerance, dMin, dMax, votingThreshold,
                                    votingRatioThreshold, maxSearchDepth,
                                    blurKernelSize, cannyThreshold1,
                                    cannyThreshold2, cannyKernelSize);

  validParams = valid;
  return valid;
}

bool StereoProcessor::compute() {
  if (validParams) {
    TicToc t_costInitialization;
    costInitialization();
    std::cout << "t_costInitialization: " << t_costInitialization.toc() << "ms"
              << std::endl;
    TicToc t_costAggregation;
    costAggregation();
    std::cout << "t_costAggregation: " << t_costAggregation.toc() << "ms"
              << std::endl;
    TicToc t_scanlineOptimization;
    scanlineOptimization();
    std::cout << "t_scanlineOptimization: " << t_scanlineOptimization.toc()
              << "ms" << std::endl;
    outlierElimination();
    regionVoting();
    properInterpolation();
    discontinuityAdjustment();
    subpixelEnhancement();
    dispComputed = true;
  }

  return validParams;
}

Mat StereoProcessor::getDisparity() const {
  return (dispComputed) ? floatDisparityMap : Mat();
}

void StereoProcessor::costInitialization() {
  Size halfCensusWin(censusWin.width / 2, censusWin.height / 2);
  bool out;
  int d, h, w, wL, wR;
  size_t imageNo;

  for (imageNo = 0; imageNo < 2; ++imageNo) {
#pragma omp parallel default(shared) private(d, h, w, wL, wR, out)             \
    num_threads(omp_get_max_threads())
#pragma omp for schedule(static)
    for (d = 0; d <= dMax - dMin; d++) {
      for (h = 0; h < imgSize.height; h++) {
        for (w = 0; w < imgSize.width; w++) {
          wL = w;
          wR = w;

          if (imageNo == 0)
            wR = w - d;
          else
            wL = w + d;

          out = wL - halfCensusWin.width < 0 ||
                wL + halfCensusWin.width >= imgSize.width ||
                wR - halfCensusWin.width < 0 ||
                wR + halfCensusWin.width >= imgSize.width ||
                h - halfCensusWin.height < 0 ||
                h + halfCensusWin.height >= imgSize.height;

          costMaps[imageNo][d].at<costType>(h, w) =
              out ? defaultBorderCost * COST_FACTOR
                  : adCensus->adCensus(wL, h, wR, h) / 2 * COST_FACTOR;
        }
      }
    }
  }

#ifdef DEBUG
  Mat disp = cost2disparity(0);
  saveDisparity<int>(disp, "01_dispLR.png");

  disp = cost2disparity(1);
  saveDisparity<int>(disp, "01_dispRL.png");
#endif
}

void StereoProcessor::costAggregation() {
  size_t imageNo, i;
  int d;

  for (imageNo = 0; imageNo < 2; ++imageNo) {

#if COST_TYPE_NOT_FLOAT
#pragma omp parallel default(shared) private(d, i)                             \
    num_threads(omp_get_max_threads())
#pragma omp for schedule(static)
    for (d = 0; d <= dMax - dMin; d++) {
      Mat currCostMap(imgSize, CV_32F);

      for (int h = 0; h < imgSize.height; h++) {
        for (int w = 0; w < imgSize.width; w++) {
          currCostMap.at<float>(h, w) =
              (float)costMaps[imageNo][d].at<costType>(h, w) / COST_FACTOR;
        }
      }

      bool horizontalFirst = true;
      for (i = 0; i < aggregatingIterations; i++) {
        aggregation->aggregation2D(currCostMap, horizontalFirst, imageNo);
        horizontalFirst = !horizontalFirst;
      }

      for (int h = 0; h < imgSize.height; h++) {
        for (int w = 0; w < imgSize.width; w++) {
          costMaps[imageNo][d].at<costType>(h, w) =
              (costType)(currCostMap.at<float>(h, w) * COST_FACTOR);
        }
      }
    }
#else
#pragma omp parallel default(shared) private(d, i)                             \
    num_threads(omp_get_max_threads())
#pragma omp for schedule(static)
    for (d = 0; d <= dMax - dMin; d++) {
      bool horizontalFirst = true;
      for (i = 0; i < aggregatingIterations; i++) {
        aggregation->aggregation2D(costMaps[imageNo][d], horizontalFirst,
                                   imageNo);
        horizontalFirst = !horizontalFirst;
        cout << "[StereoProcessor] aggregation iteration no. " << i
             << ", disparity no. " << d << " for image no. " << imageNo << endl;
      }
    }
#endif
  }

#ifdef DEBUG
  Mat disp = cost2disparity(0);
  saveDisparity<int>(disp, "02_dispLR_agg.png");

  disp = cost2disparity(1);
  saveDisparity<int>(disp, "02_dispRL_agg.png");
#endif
}

void StereoProcessor::scanlineOptimization() {
  ScanlineOptimization sO(images[0], images[1], dMin, dMax, colorDifference,
                          pi1, pi2);
  int imageNo;

#pragma omp parallel default(shared) private(imageNo)                          \
    num_threads(omp_get_max_threads())
#pragma omp for schedule(static)
  for (imageNo = 0; imageNo < 2; ++imageNo) {

    sO.optimization(&costMaps[imageNo], (imageNo == 1));
  }

#ifdef DEBUG
  Mat disp = cost2disparity(0);
  saveDisparity<int>(disp, "03_dispLR_so.png");

  disp = cost2disparity(1);
  saveDisparity<int>(disp, "03_dispRL_so.png");
#endif
}

void StereoProcessor::outlierElimination() {

  Mat disp0 = cost2disparity(0);
  Mat disp1 = cost2disparity(1);

  disparityMap = dispRef->outlierElimination(disp0, disp1);

#ifdef DEBUG
  saveDisparity<int>(disparityMap, "04_dispBoth_oe.png");
#endif
}

void StereoProcessor::regionVoting() {
  vector<Mat> upLimits, downLimits, leftLimits, rightLimits;

  aggregation->getLimits(upLimits, downLimits, leftLimits, rightLimits);

  bool horizontalFirst = false;
  for (int i = 0; i < 5; i++) {
    dispRef->regionVoting(disparityMap, upLimits, downLimits, leftLimits,
                          rightLimits, horizontalFirst);
    horizontalFirst = ~horizontalFirst;
  }

#ifdef DEBUG
  saveDisparity<int>(disparityMap, "05_dispBoth_rv.png");
#endif
}

void StereoProcessor::properInterpolation() {
  dispRef->properInterpolation(disparityMap, images[0]);

#ifdef DEBUG
  saveDisparity<int>(disparityMap, "06_dispBoth_pi.png");
#endif
}

void StereoProcessor::discontinuityAdjustment() {
  dispRef->discontinuityAdjustment(disparityMap, costMaps);

#ifdef DEBUG
  saveDisparity<int>(disparityMap, "07_dispBoth_da.png");
#endif
}

void StereoProcessor::subpixelEnhancement() {
  floatDisparityMap = dispRef->subpixelEnhancement(disparityMap, costMaps);

#ifdef DEBUG
  saveDisparity<float>(floatDisparityMap, "08_dispBoth_se.png");
#endif
}

Mat StereoProcessor::cost2disparity(int imageNo) {
  Mat disp(imgSize, CV_32S);
  Mat lowCost(imgSize, COST_MAP_TYPE,
              Scalar(std::numeric_limits<costType>::max()));

  for (int d = 0; d <= dMax - dMin; d++) {
    for (size_t h = 0; h < imgSize.height; h++) {
      for (size_t w = 0; w < imgSize.width; w++) {
        if (lowCost.at<costType>(h, w) >
            costMaps[imageNo][d].at<costType>(h, w)) {
          lowCost.at<costType>(h, w) = costMaps[imageNo][d].at<costType>(h, w);
          disp.at<int>(h, w) = d + dMin;
        }
      }
    }
  }

  return disp;
}
