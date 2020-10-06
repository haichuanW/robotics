#ifndef ADCENSUS_STEREO_TYPES_H_
#define ADCENSUS_STEREO_TYPES_H_

#include <cstdint>
#include <limits>
#include <vector>
using std::pair;
using std::vector;
using namespace std;

#ifndef SAFE_DELETE
#define SAFE_DELETE(P)      \
    {                       \
        if (P) delete[](P); \
        (P) = nullptr;      \
    }
#endif

typedef uint8_t uint8;

constexpr auto Invalid_Float = std::numeric_limits<float>::infinity();

constexpr auto Large_Float = 99999.0f;
constexpr auto Small_Float = -99999.0f;

enum CensusSize { Census5x5 = 0, Census9x7 };

struct ADCensusOption {
    int min_disparity;
    int max_disparity;

    int lambda_ad;
    int lambda_census;
    int cross_L1;
    int cross_L2;
    int cross_t1;
    int cross_t2;
    float so_p1;
    float so_p2;
    int so_tso;
    int irv_ts;
    float irv_th;

    float lrcheck_thres;

    bool do_lr_check;
    bool do_filling;
    bool do_discontinuity_adjustment;

    ADCensusOption()
        : min_disparity(0),
          max_disparity(64),
          lambda_ad(10),
          lambda_census(30),
          cross_L1(34),
          cross_L2(17),
          cross_t1(20),
          cross_t2(6),
          so_p1(1.0f),
          so_p2(3.0f),
          so_tso(15),
          irv_ts(20),
          irv_th(0.4f),
          lrcheck_thres(1.0f),
          do_lr_check(true),
          do_filling(true),
          do_discontinuity_adjustment(false){};
};

struct ADColor {
    uint8 r, g, b;
    ADColor() : r(0), g(0), b(0) {}
    ADColor(uint8 _b, uint8 _g, uint8 _r) {
        r = _r;
        g = _g;
        b = _b;
    }
};

#endif
