#include "ADCensusStereo.h"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std::chrono;

ADCensusStereo::ADCensusStereo()
    : width_(0), height_(0), img_left_(nullptr), img_right_(nullptr),
      disp_left_(nullptr), disp_right_(nullptr), is_initialized_(true) {}

ADCensusStereo::~ADCensusStereo() {
  Release();
  is_initialized_ = false;
}

bool ADCensusStereo::Initialize(const int &width, const int &height,
                                const ADCensusOption &option) {
  width_ = width;
  height_ = height;
  option_ = option;

  if (width <= 0 || height <= 0) {
    return false;
  }

  const int img_size = width_ * height_;
  const int disp_range = option_.max_disparity - option_.min_disparity;
  if (disp_range <= 0) {
    return false;
  }

  disp_left_ = new float[img_size];
  disp_right_ = new float[img_size];

  if (!cost_computer_.Initialize(width_, height_, option_.min_disparity,
                                 option_.max_disparity)) {
    is_initialized_ = false;
    return is_initialized_;
  }

  if (!aggregator_.Initialize(width_, height_, option_.min_disparity,
                              option_.max_disparity)) {
    is_initialized_ = false;
    return is_initialized_;
  }

  if (!refiner_.Initialize(width_, height_)) {
    is_initialized_ = false;
    return is_initialized_;
  }

  is_initialized_ = disp_left_ && disp_right_;

  return is_initialized_;
}

bool ADCensusStereo::Match(const cv::Mat &img_left, const cv::Mat &img_right,
                           cv::Mat &disp_left) {
  if (!is_initialized_) {
    return false;
  }

  img_left_ = img_left.data;
  img_right_ = img_right.data;

  auto start = steady_clock::now();

  ComputeCost();

  auto end = steady_clock::now();
  auto tt = duration_cast<milliseconds>(end - start);
  printf("computing cost! timing :	%lf s\n", tt.count() / 1000.0);
  start = steady_clock::now();

  CostAggregation();

  end = steady_clock::now();
  tt = duration_cast<milliseconds>(end - start);
  printf("cost aggregating! timing :	%lf s\n", tt.count() / 1000.0);
  start = steady_clock::now();

  ScanlineOptimize();

  end = steady_clock::now();
  tt = duration_cast<milliseconds>(end - start);
  printf("scanline optimizing! timing :	%lf s\n", tt.count() / 1000.0);
  start = steady_clock::now();

  ComputeDisparity();
  ComputeDisparityRight();

  end = steady_clock::now();
  tt = duration_cast<milliseconds>(end - start);
  printf("computing disparities! timing :	%lf s\n", tt.count() / 1000.0);
  start = steady_clock::now();

  MultiStepRefine();

  end = steady_clock::now();
  tt = duration_cast<milliseconds>(end - start);
  printf("multistep refining! timing :	%lf s\n", tt.count() / 1000.0);
  start = steady_clock::now();

  memcpy(disp_left.data, disp_left_, height_ * width_ * sizeof(float));

  end = steady_clock::now();
  tt = duration_cast<milliseconds>(end - start);
  printf("output disparities! timing :	%lf s\n", tt.count() / 1000.0);

  return true;
}

bool ADCensusStereo::Reset(const size_t &width, const size_t &height,
                           const ADCensusOption &option) {
  Release();

  is_initialized_ = false;

  return Initialize(width, height, option);
}

void ADCensusStereo::ComputeCost() {
  cost_computer_.SetData(img_left_, img_right_);
  cost_computer_.SetParams(option_.lambda_ad, option_.lambda_census);
  cost_computer_.Compute();
}

void ADCensusStereo::CostAggregation() {
  aggregator_.SetData(img_left_, img_right_, cost_computer_.get_cost_ptr());
  aggregator_.SetParams(option_.cross_L1, option_.cross_L2, option_.cross_t1,
                        option_.cross_t2);
  aggregator_.Aggregate(4);
}

void ADCensusStereo::ScanlineOptimize() {
  scan_line_.SetData(img_left_, img_right_, cost_computer_.get_cost_ptr(),
                     aggregator_.get_cost_ptr());
  scan_line_.SetParam(width_, height_, option_.min_disparity,
                      option_.max_disparity, option_.so_p1, option_.so_p2,
                      option_.so_tso);
  scan_line_.Optimize();
}

void ADCensusStereo::MultiStepRefine() {
  refiner_.SetData(img_left_, aggregator_.get_cost_ptr(),
                   aggregator_.get_arms_ptr(), disp_left_, disp_right_);
  refiner_.SetParam(option_.min_disparity, option_.max_disparity,
                    option_.irv_ts, option_.irv_th, option_.lrcheck_thres,
                    option_.do_lr_check, option_.do_filling, option_.do_filling,
                    option_.do_discontinuity_adjustment);
  refiner_.Refine();
}

void ADCensusStereo::ComputeDisparity() {
  const int &min_disparity = option_.min_disparity;
  const int &max_disparity = option_.max_disparity;
  const int disp_range = max_disparity - min_disparity;
  if (disp_range <= 0) {
    return;
  }

  const auto disparity = disp_left_;
  const auto cost_ptr = aggregator_.get_cost_ptr();

  const int width = width_;
  const int height = height_;

  std::vector<float> cost_local(disp_range);

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      float min_cost = Large_Float;
      int best_disparity = 0;

      for (int d = min_disparity; d < max_disparity; d++) {
        const int d_idx = d - min_disparity;
        const auto &cost = cost_local[d_idx] =
            cost_ptr[i * width * disp_range + j * disp_range + d_idx];
        if (min_cost > cost) {
          min_cost = cost;
          best_disparity = d;
        }
      }
      if (best_disparity == min_disparity ||
          best_disparity == max_disparity - 1) {
        disparity[i * width + j] = Invalid_Float;
        continue;
      }
      const int idx_1 = best_disparity - 1 - min_disparity;
      const int idx_2 = best_disparity + 1 - min_disparity;
      const float cost_1 = cost_local[idx_1];
      const float cost_2 = cost_local[idx_2];
      const float denom = cost_1 + cost_2 - 2 * min_cost;
      if (denom != 0.0f) {
        disparity[i * width + j] = static_cast<float>(best_disparity) +
                                   (cost_1 - cost_2) / (denom * 2.0f);
      } else {
        disparity[i * width + j] = static_cast<float>(best_disparity);
      }
    }
  }
}

void ADCensusStereo::ComputeDisparityRight() {
  const int &min_disparity = option_.min_disparity;
  const int &max_disparity = option_.max_disparity;
  const int disp_range = max_disparity - min_disparity;
  if (disp_range <= 0) {
    return;
  }

  const auto disparity = disp_right_;
  const auto cost_ptr = aggregator_.get_cost_ptr();

  const int width = width_;
  const int height = height_;

  std::vector<float> cost_local(disp_range);

  // cost(xr,yr,d) = cost(xr+d,yl,d)
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      float min_cost = Large_Float;
      int best_disparity = 0;

      for (int d = min_disparity; d < max_disparity; d++) {
        const int d_idx = d - min_disparity;
        const int col_left = j + d;
        if (col_left >= 0 && col_left < width) {
          const auto &cost = cost_local[d_idx] =
              cost_ptr[i * width * disp_range + col_left * disp_range + d_idx];
          if (min_cost > cost) {
            min_cost = cost;
            best_disparity = d;
          }
        } else {
          cost_local[d_idx] = Large_Float;
        }
      }

      if (best_disparity == min_disparity ||
          best_disparity == max_disparity - 1) {
        disparity[i * width + j] = best_disparity;
        continue;
      }

      const int idx_1 = best_disparity - 1 - min_disparity;
      const int idx_2 = best_disparity + 1 - min_disparity;
      const float cost_1 = cost_local[idx_1];
      const float cost_2 = cost_local[idx_2];
      const float denom = cost_1 + cost_2 - 2 * min_cost;
      if (denom != 0.0f) {
        disparity[i * width + j] = static_cast<float>(best_disparity) +
                                   (cost_1 - cost_2) / (denom * 2.0f);
      } else {
        disparity[i * width + j] = static_cast<float>(best_disparity);
      }
    }
  }
}

void ADCensusStereo::Release() {
  SAFE_DELETE(disp_left_);
  SAFE_DELETE(disp_right_);
}
