#include "cost_computor.h"
#include <cmath>
#include "adcensus_util.h"

CostComputor::CostComputor()
    : width_(0),
      height_(0),
      img_left_(nullptr),
      img_right_(nullptr),
      lambda_ad_(0),
      lambda_census_(0),
      min_disparity_(0),
      max_disparity_(0),
      is_initialized_(false) {}

CostComputor::~CostComputor() {}

bool CostComputor::Initialize(const int &width, const int &height, const int &min_disparity,
                              const int &max_disparity) {
    width_ = width;
    height_ = height;
    min_disparity_ = min_disparity;
    max_disparity_ = max_disparity;

    const int img_size = width_ * height_;
    const int disp_range = max_disparity_ - min_disparity_;
    if (img_size <= 0 || disp_range <= 0) {
        is_initialized_ = false;
        return false;
    }

    gray_left_.resize(img_size);
    gray_right_.resize(img_size);
    census_left_.resize(img_size, 0);
    census_right_.resize(img_size, 0);
    cost_init_.resize(img_size * disp_range);

    is_initialized_ = !gray_left_.empty() && !gray_right_.empty() && !census_left_.empty() &&
                      !census_right_.empty() && !cost_init_.empty();
    return is_initialized_;
}

void CostComputor::SetData(const uint8 *img_left, const uint8 *img_right) {
    img_left_ = img_left;
    img_right_ = img_right;
}

void CostComputor::SetParams(const int &lambda_ad, const int &lambda_census) {
    lambda_ad_ = lambda_ad;
    lambda_census_ = lambda_census;
}

void CostComputor::ComputeGray() {
    for (int n = 0; n < 2; n++) {
        const auto color = (n == 0) ? img_left_ : img_right_;
        auto &gray = (n == 0) ? gray_left_ : gray_right_;
        for (int y = 0; y < height_; y++) {
            for (int x = 0; x < width_; x++) {
                const auto b = color[y * width_ * 3 + 3 * x];
                const auto g = color[y * width_ * 3 + 3 * x + 1];
                const auto r = color[y * width_ * 3 + 3 * x + 2];
                gray[y * width_ + x] = uint8(r * 0.299 + g * 0.587 + b * 0.114);
            }
        }
    }
}

void CostComputor::CensusTransform() {
    adcensus_util::census_transform_9x7(&gray_left_[0], census_left_, width_, height_);
    adcensus_util::census_transform_9x7(&gray_right_[0], census_right_, width_, height_);
}

void CostComputor::ComputeCost() {
    const int disp_range = max_disparity_ - min_disparity_;

    const auto lambda_ad = lambda_ad_;
    const auto lambda_census = lambda_census_;

    for (int y = 0; y < height_; y++) {
        for (int x = 0; x < width_; x++) {
            const auto bl = img_left_[y * width_ * 3 + 3 * x];
            const auto gl = img_left_[y * width_ * 3 + 3 * x + 1];
            const auto rl = img_left_[y * width_ * 3 + 3 * x + 2];
            const auto &census_val_l = census_left_[y * width_ + x];
            for (int d = min_disparity_; d < max_disparity_; d++) {
                auto &cost =
                    cost_init_[y * width_ * disp_range + x * disp_range + (d - min_disparity_)];
                const int xr = x - d;
                if (xr < 0 || xr >= width_) {
                    cost = 1.0f;
                    continue;
                }

                const auto br = img_right_[y * width_ * 3 + 3 * xr];
                const auto gr = img_right_[y * width_ * 3 + 3 * xr + 1];
                const auto rr = img_right_[y * width_ * 3 + 3 * xr + 2];
                const float cost_ad = (abs(bl - br) + abs(gl - gr) + abs(rl - rr)) / 3.0f;

                const auto &census_val_r = census_right_[y * width_ + xr];
                const float cost_census =
                    static_cast<float>(adcensus_util::Hamming64(census_val_l, census_val_r));

                cost = 1 - exp(-cost_ad / lambda_ad) + 1 - exp(-cost_census / lambda_census);
            }
        }
    }
}

void CostComputor::Compute() {
    if (!is_initialized_) {
        return;
    }

    ComputeGray();

    CensusTransform();

    ComputeCost();
}

float *CostComputor::get_cost_ptr() {
    if (!cost_init_.empty()) {
        return &cost_init_[0];
    } else {
        return nullptr;
    }
}
