#include "multistep_refiner.h"
#include <cmath>
#include <cstring>
#include "adcensus_util.h"
MultiStepRefiner::MultiStepRefiner()
    : width_(0),
      height_(0),
      img_left_(nullptr),
      cost_(nullptr),
      cross_arms_(nullptr),
      disp_left_(nullptr),
      disp_right_(nullptr),
      min_disparity_(0),
      max_disparity_(0),
      irv_ts_(0),
      irv_th_(0),
      lrcheck_thres_(0),
      do_lr_check_(false),
      do_region_voting_(false),
      do_interpolating_(false),
      do_discontinuity_adjustment_(false) {}

MultiStepRefiner::~MultiStepRefiner() {}

bool MultiStepRefiner::Initialize(const int &width, const int &height) {
    width_ = width;
    height_ = height;
    if (width_ <= 0 || height_ <= 0) {
        return false;
    }

    vec_edge_left_.clear();
    vec_edge_left_.resize(width * height);

    return true;
}

void MultiStepRefiner::SetData(const uint8 *img_left, float *cost, const CrossArm *cross_arms,
                               float *disp_left, float *disp_right) {
    img_left_ = img_left;
    cost_ = cost;
    cross_arms_ = cross_arms;
    disp_left_ = disp_left;
    disp_right_ = disp_right;
}

void MultiStepRefiner::SetParam(const int &min_disparity, const int &max_disparity,
                                const int &irv_ts, const float &irv_th, const float &lrcheck_thres,
                                const bool &do_lr_check, const bool &do_region_voting,
                                const bool &do_interpolating,
                                const bool &do_discontinuity_adjustment) {
    min_disparity_ = min_disparity;
    max_disparity_ = max_disparity;
    irv_ts_ = irv_ts;
    irv_th_ = irv_th;
    lrcheck_thres_ = lrcheck_thres;
    do_lr_check_ = do_lr_check;
    do_region_voting_ = do_region_voting;
    do_interpolating_ = do_interpolating;
    do_discontinuity_adjustment_ = do_discontinuity_adjustment;
}

void MultiStepRefiner::Refine() {
    if (width_ <= 0 || height_ <= 0 || disp_left_ == nullptr || disp_right_ == nullptr ||
        cost_ == nullptr || cross_arms_ == nullptr) {
        return;
    }

    // step1: outlier detection
    if (do_lr_check_) {
        OutlierDetection();
    }
    // step2: iterative region voting
    if (do_region_voting_) {
        IterativeRegionVoting();
    }
    // step3: proper interpolation
    if (do_interpolating_) {
        ProperInterpolation();
    }
    // step4: discontinuities adjustment
    if (do_discontinuity_adjustment_) {
        DepthDiscontinuityAdjustment();
    }

    // median filter
    adcensus_util::MedianFilter(disp_left_, disp_left_, width_, height_, 3);
}

void MultiStepRefiner::OutlierDetection() {
    const int width = width_;
    const int height = height_;

    const float &threshold = lrcheck_thres_;

    auto &occlusions = occlusions_;
    auto &mismatches = mismatches_;
    occlusions.clear();
    mismatches.clear();

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            auto &disp = disp_left_[y * width + x];
            if (disp == Invalid_Float) {
                mismatches.emplace_back(x, y);
                continue;
            }

            const auto col_right = lround(x - disp);
            if (col_right >= 0 && col_right < width) {
                const auto &disp_r = disp_right_[y * width + col_right];
                if (abs(disp - disp_r) > threshold) {
                    // if(disp_rl > disp)
                    //		pixel in occlusions
                    // else
                    //		pixel in mismatches
                    const int col_rl = lround(col_right + disp_r);
                    if (col_rl > 0 && col_rl < width) {
                        const auto &disp_l = disp_left_[y * width + col_rl];
                        if (disp_l > disp) {
                            occlusions.emplace_back(x, y);
                        } else {
                            mismatches.emplace_back(x, y);
                        }
                    } else {
                        mismatches.emplace_back(x, y);
                    }

                    disp = Invalid_Float;
                }
            } else {
                disp = Invalid_Float;
                mismatches.emplace_back(x, y);
            }
        }
    }
}

void MultiStepRefiner::IterativeRegionVoting() {
    const int width = width_;

    const auto disp_range = max_disparity_ - min_disparity_;
    if (disp_range <= 0) {
        return;
    }
    const auto arms = cross_arms_;

    vector<int> histogram(disp_range, 0);

    const int num_iters = 5;

    for (int it = 0; it < num_iters; it++) {
        for (int k = 0; k < 2; k++) {
            auto &trg_pixels = (k == 0) ? mismatches_ : occlusions_;
            for (auto &pix : trg_pixels) {
                const int &x = pix.first;
                const int &y = pix.second;
                auto &disp = disp_left_[y * width + x];
                if (disp != Invalid_Float) {
                    continue;
                }

                // init histogram
                memset(&histogram[0], 0, disp_range * sizeof(int));

                auto &arm = arms[y * width + x];
                for (int t = -arm.top; t <= arm.bottom; t++) {
                    const int &yt = y + t;
                    auto &arm2 = arms[yt * width_ + x];
                    for (int s = -arm2.left; s <= arm2.right; s++) {
                        const auto &d = disp_left_[yt * width + x + s];
                        if (d != Invalid_Float) {
                            const auto di = lround(d);
                            histogram[di - min_disparity_]++;
                        }
                    }
                }
                int best_disp = 0, count = 0;
                int max_ht = 0;
                for (int d = 0; d < disp_range; d++) {
                    const auto &h = histogram[d];
                    if (max_ht < h) {
                        max_ht = h;
                        best_disp = d;
                    }
                    count += h;
                }

                if (max_ht > 0) {
                    if (count > irv_ts_ && max_ht * 1.0f / count > irv_th_) {
                        disp = best_disp + min_disparity_;
                    }
                }
            }
            for (auto it = trg_pixels.begin(); it != trg_pixels.end();) {
                const int x = it->first;
                const int y = it->second;
                if (disp_left_[y * width + x] != Invalid_Float) {
                    it = trg_pixels.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }
}

void MultiStepRefiner::ProperInterpolation() {
    const int width = width_;
    const int height = height_;

    const float pi = 3.1415926f;
    const int max_search_length = std::max(abs(max_disparity_), abs(min_disparity_));

    std::vector<pair<int, float>> disp_collects;
    for (int k = 0; k < 2; k++) {
        auto &trg_pixels = (k == 0) ? mismatches_ : occlusions_;
        if (trg_pixels.empty()) {
            continue;
        }
        std::vector<float> fill_disps(trg_pixels.size());

        for (auto n = 0u; n < trg_pixels.size(); n++) {
            auto &pix = trg_pixels[n];
            const int x = pix.first;
            const int y = pix.second;

            disp_collects.clear();
            double ang = 0.0;
            for (int s = 0; s < 16; s++) {
                const auto sina = sin(ang);
                const auto cosa = cos(ang);
                for (int m = 1; m < max_search_length; m++) {
                    const int yy = lround(y + m * sina);
                    const int xx = lround(x + m * cosa);
                    if (yy < 0 || yy >= height || xx < 0 || xx >= width) {
                        break;
                    }
                    const auto &d = disp_left_[yy * width + xx];
                    if (d != Invalid_Float) {
                        disp_collects.emplace_back(yy * width * 3 + 3 * xx, d);
                        break;
                    }
                }
                ang += pi / 16;
            }
            if (disp_collects.empty()) {
                continue;
            }

            if (k == 0) {
                int min_dist = 9999;
                float d = 0.0f;
                const auto color =
                    ADColor(img_left_[y * width * 3 + 3 * x], img_left_[y * width * 3 + 3 * x + 1],
                            img_left_[y * width * 3 + 3 * x + 2]);
                for (auto &dc : disp_collects) {
                    const auto color2 = ADColor(img_left_[dc.first], img_left_[dc.first + 1],
                                                img_left_[dc.first + 2]);
                    const auto dist =
                        abs(color.r - color2.r) + abs(color.g - color2.g) + abs(color.b - color2.b);
                    if (min_dist > dist) {
                        min_dist = dist;
                        d = dc.second;
                    }
                }
                fill_disps[n] = d;
            } else {
                float min_disp = Large_Float;
                for (auto &dc : disp_collects) {
                    min_disp = std::min(min_disp, dc.second);
                }
                fill_disps[n] = min_disp;
            }
        }
        for (auto n = 0u; n < trg_pixels.size(); n++) {
            auto &pix = trg_pixels[n];
            const int x = pix.first;
            const int y = pix.second;
            disp_left_[y * width + x] = fill_disps[n];
        }
    }
}

void MultiStepRefiner::DepthDiscontinuityAdjustment() {
    const int width = width_;
    const int height = height_;
    const auto disp_range = max_disparity_ - min_disparity_;
    if (disp_range <= 0) {
        return;
    }

    const float edge_thres = 5.0f;
    EdgeDetect(&vec_edge_left_[0], disp_left_, width, height, edge_thres);

    for (int y = 0; y < height; y++) {
        for (int x = 1; x < width - 1; x++) {
            const auto &e_label = vec_edge_left_[y * width + x];
            if (e_label == 1) {
                const auto disp_ptr = disp_left_ + y * width;
                float &d = disp_ptr[x];
                if (d != Invalid_Float) {
                    const int &di = lround(d);
                    const auto cost_ptr = cost_ + y * width * disp_range + x * disp_range;
                    float c0 = cost_ptr[di];

                    bool adjust = false;
                    for (int k = 0; k < 2; k++) {
                        const int x2 = (k == 0) ? x - 1 : x + 1;
                        const float &d2 = disp_ptr[x2];
                        const int &d2i = lround(d2);
                        if (d2 != Invalid_Float) {
                            const auto &c =
                                (k == 0) ? cost_ptr[-disp_range + d2i] : cost_ptr[disp_range + d2i];
                            if (c < c0) {
                                d = d2;
                                c0 = c;
                                adjust = true;
                            }
                        }
                    }
                }
            }
        }
    }
}

void MultiStepRefiner::EdgeDetect(uint8 *edge_mask, const float *disp_ptr, const int &width,
                                  const int &height, const float threshold) {
    memset(edge_mask, 0, width * height * sizeof(uint8));
    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            const auto grad_x =
                (-disp_ptr[(y - 1) * width + x - 1] + disp_ptr[(y - 1) * width + x + 1]) +
                (-2 * disp_ptr[y * width + x - 1] + 2 * disp_ptr[y * width + x + 1]) +
                (-disp_ptr[(y + 1) * width + x - 1] + disp_ptr[(y + 1) * width + x + 1]);
            const auto grad_y =
                (-disp_ptr[(y - 1) * width + x - 1] - 2 * disp_ptr[(y - 1) * width + x] -
                 disp_ptr[(y - 1) * width + x + 1]) +
                (disp_ptr[(y + 1) * width + x - 1] + 2 * disp_ptr[(y + 1) * width + x] +
                 disp_ptr[(y + 1) * width + x + 1]);
            const auto grad = abs(grad_x) + abs(grad_y);
            if (grad > threshold) {
                edge_mask[y * width + x] = 1;
            }
        }
    }
}
