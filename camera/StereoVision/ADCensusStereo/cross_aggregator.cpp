#include "cross_aggregator.h"
#include <cstring>

CrossAggregator::CrossAggregator()
    : width_(0),
      height_(0),
      img_left_(nullptr),
      img_right_(nullptr),
      cost_init_(nullptr),
      cross_L1_(0),
      cross_L2_(0),
      cross_t1_(0),
      cross_t2_(0),
      min_disparity_(0),
      max_disparity_(0),
      is_initialized_(false) {}

CrossAggregator::~CrossAggregator() {}

bool CrossAggregator::Initialize(const int &width, const int &height, const int &min_disparity,
                                 const int &max_disparity) {
    width_ = width;
    height_ = height;
    min_disparity_ = min_disparity;
    max_disparity_ = max_disparity;

    const int img_size = width_ * height_;
    const int disp_range = max_disparity_ - min_disparity_;
    if (img_size <= 0 || disp_range <= 0) {
        is_initialized_ = false;
        return is_initialized_;
    }

    vec_cross_arms_.clear();
    vec_cross_arms_.resize(img_size);

    vec_cost_tmp_[0].clear();
    vec_cost_tmp_[0].resize(img_size);
    vec_cost_tmp_[1].clear();
    vec_cost_tmp_[1].resize(img_size);

    vec_sup_count_[0].clear();
    vec_sup_count_[0].resize(img_size);
    vec_sup_count_[1].clear();
    vec_sup_count_[1].resize(img_size);
    vec_sup_count_tmp_.clear();
    vec_sup_count_tmp_.resize(img_size);

    cost_aggr_.resize(img_size * disp_range);

    is_initialized_ = !vec_cross_arms_.empty() && !vec_cost_tmp_[0].empty() &&
                      !vec_cost_tmp_[1].empty() && !vec_sup_count_[0].empty() &&
                      !vec_sup_count_[1].empty() && !vec_sup_count_tmp_.empty() &&
                      !cost_aggr_.empty();
    return is_initialized_;
}

void CrossAggregator::SetData(const uint8 *img_left, const uint8 *img_right,
                              const float *cost_init) {
    img_left_ = img_left;
    img_right_ = img_right;
    cost_init_ = cost_init;
}

void CrossAggregator::SetParams(const int &cross_L1, const int &cross_L2, const int &cross_t1,
                                const int &cross_t2) {
    cross_L1_ = cross_L1;
    cross_L2_ = cross_L2;
    cross_t1_ = cross_t1;
    cross_t2_ = cross_t2;
}

void CrossAggregator::BuildArms() {
    for (int y = 0; y < height_; y++) {
        for (int x = 0; x < width_; x++) {
            CrossArm &arm = vec_cross_arms_[y * width_ + x];
            FindHorizontalArm(x, y, arm.left, arm.right);
            FindVerticalArm(x, y, arm.top, arm.bottom);
        }
    }
}

void CrossAggregator::Aggregate(const int &num_iters) {
    if (!is_initialized_) {
        return;
    }

    const int disp_range = max_disparity_ - min_disparity_;

    BuildArms();

    bool horizontal_first = true;

    ComputeSupPixelCount();

    std::memcpy(&cost_aggr_[0], cost_init_, width_ * height_ * disp_range * sizeof(float));

    for (int k = 0; k < num_iters; k++) {
        for (int d = min_disparity_; d < max_disparity_; d++) {
            AggregateInArms(d, horizontal_first);
        }
        horizontal_first = !horizontal_first;
    }
}

CrossArm *CrossAggregator::get_arms_ptr() { return &vec_cross_arms_[0]; }

float *CrossAggregator::get_cost_ptr() {
    if (!cost_aggr_.empty()) {
        return &cost_aggr_[0];
    } else {
        return nullptr;
    }
}

void CrossAggregator::FindHorizontalArm(const int &x, const int &y, uint8 &left,
                                        uint8 &right) const {
    const auto img0 = img_left_ + y * width_ * 3 + 3 * x;
    const ADColor color0(img0[0], img0[1], img0[2]);

    left = right = 0;
    int dir = -1;
    for (int k = 0; k < 2; k++) {
        auto img = img0 + dir * 3;
        auto color_last = color0;
        int xn = x + dir;
        for (int n = 0; n < std::min(cross_L1_, MAX_ARM_LENGTH); n++) {
            if (k == 0) {
                if (xn < 0) {
                    break;
                }
            } else {
                if (xn == width_) {
                    break;
                }
            }

            const ADColor color(img[0], img[1], img[2]);

            const int color_dist1 = ColorDist(color, color0);
            if (color_dist1 >= cross_t1_) {
                break;
            }

            if (n > 0) {
                const int color_dist2 = ColorDist(color, color_last);
                if (color_dist2 >= cross_t1_) {
                    break;
                }
            }

            if (n + 1 > cross_L2_) {
                if (color_dist1 >= cross_t2_) {
                    break;
                }
            }

            if (k == 0) {
                left++;
            } else {
                right++;
            }
            color_last = color;
            xn += dir;
            img += dir * 3;
        }
        dir = -dir;
    }
}

void CrossAggregator::FindVerticalArm(const int &x, const int &y, uint8 &top, uint8 &bottom) const {
    const auto img0 = img_left_ + y * width_ * 3 + 3 * x;
    const ADColor color0(img0[0], img0[1], img0[2]);

    top = bottom = 0;
    int dir = -1;
    for (int k = 0; k < 2; k++) {
        auto img = img0 + dir * width_ * 3;
        auto color_last = color0;
        int yn = y + dir;
        for (int n = 0; n < std::min(cross_L1_, MAX_ARM_LENGTH); n++) {
            if (k == 0) {
                if (yn < 0) {
                    break;
                }
            } else {
                if (yn == height_) {
                    break;
                }
            }

            const ADColor color(img[0], img[1], img[2]);

            const int color_dist1 = ColorDist(color, color0);
            if (color_dist1 >= cross_t1_) {
                break;
            }

            if (n > 0) {
                const int color_dist2 = ColorDist(color, color_last);
                if (color_dist2 >= cross_t1_) {
                    break;
                }
            }

            if (n + 1 > cross_L2_) {
                if (color_dist1 >= cross_t2_) {
                    break;
                }
            }

            if (k == 0) {
                top++;
            } else {
                bottom++;
            }
            color_last = color;
            yn += dir;
            img += dir * width_ * 3;
        }
        dir = -dir;
    }
}

void CrossAggregator::ComputeSupPixelCount() {
    bool horizontal_first = true;
    for (int n = 0; n < 2; n++) {
        // n=0 : horizontal_first; n=1 : vertical_first
        const int id = horizontal_first ? 0 : 1;
        for (int k = 0; k < 2; k++) {
            // k=0 : pass1; k=1 : pass2
            for (int y = 0; y < height_; y++) {
                for (int x = 0; x < width_; x++) {
                    auto &arm = vec_cross_arms_[y * width_ + x];
                    int count = 0;
                    if (horizontal_first) {
                        if (k == 0) {
                            // horizontal
                            for (int t = -arm.left; t <= arm.right; t++) {
                                count++;
                            }
                        } else {
                            // vertical
                            for (int t = -arm.top; t <= arm.bottom; t++) {
                                count += vec_sup_count_tmp_[(y + t) * width_ + x];
                            }
                        }
                    } else {
                        if (k == 0) {
                            // vertical
                            for (int t = -arm.top; t <= arm.bottom; t++) {
                                count++;
                            }
                        } else {
                            // horizontal
                            for (int t = -arm.left; t <= arm.right; t++) {
                                count += vec_sup_count_tmp_[y * width_ + x + t];
                            }
                        }
                    }
                    if (k == 0) {
                        vec_sup_count_tmp_[y * width_ + x] = count;
                    } else {
                        vec_sup_count_[id][y * width_ + x] = count;
                    }
                }
            }
        }
        horizontal_first = !horizontal_first;
    }
}

void CrossAggregator::AggregateInArms(const int &disparity, const bool &horizontal_first) {
    if (disparity < min_disparity_ || disparity >= max_disparity_) {
        return;
    }
    const auto disp = disparity - min_disparity_;
    const int disp_range = max_disparity_ - min_disparity_;
    if (disp_range <= 0) {
        return;
    }

    for (int y = 0; y < height_; y++) {
        for (int x = 0; x < width_; x++) {
            vec_cost_tmp_[0][y * width_ + x] =
                cost_aggr_[y * width_ * disp_range + x * disp_range + disp];
        }
    }

    const int ct_id = horizontal_first ? 0 : 1;
    for (int k = 0; k < 2; k++) {
        // k==0: pass1
        // k==1: pass2
        for (int y = 0; y < height_; y++) {
            for (int x = 0; x < width_; x++) {
                auto &arm = vec_cross_arms_[y * width_ + x];
                float cost = 0.0f;
                if (horizontal_first) {
                    if (k == 0) {
                        // horizontal
                        for (int t = -arm.left; t <= arm.right; t++) {
                            cost += vec_cost_tmp_[0][y * width_ + x + t];
                        }
                    } else {
                        // vertical
                        for (int t = -arm.top; t <= arm.bottom; t++) {
                            cost += vec_cost_tmp_[1][(y + t) * width_ + x];
                        }
                    }
                } else {
                    if (k == 0) {
                        // vertical
                        for (int t = -arm.top; t <= arm.bottom; t++) {
                            cost += vec_cost_tmp_[0][(y + t) * width_ + x];
                        }
                    } else {
                        // horizontal
                        for (int t = -arm.left; t <= arm.right; t++) {
                            cost += vec_cost_tmp_[1][y * width_ + x + t];
                        }
                    }
                }
                if (k == 0) {
                    vec_cost_tmp_[1][y * width_ + x] = cost;
                } else {
                    cost_aggr_[y * width_ * disp_range + x * disp_range + disp] =
                        cost / vec_sup_count_[ct_id][y * width_ + x];
                }
            }
        }
    }
}
