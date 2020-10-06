#include "sgm_types.h"

#ifndef SAFE_DELETE
#define SAFE_DELETE(P)                                                         \
  {                                                                            \
    if (P)                                                                     \
      delete[](P);                                                             \
    (P) = nullptr;                                                             \
  }
#endif

namespace sgm_util {

void census_transform_5x5(const uint8 *source, uint32 *census,
                          const sint32 &width, const sint32 &height);
void census_transform_9x7(const uint8 *source, uint64 *census,
                          const sint32 &width, const sint32 &height);
uint8 Hamming32(const uint32 &x, const uint32 &y);
uint8 Hamming64(const uint64 &x, const uint64 &y);

void CostAggregateLeftRight(const uint8 *img_data, const sint32 &width,
                            const sint32 &height, const sint32 &min_disparity,
                            const sint32 &max_disparity, const sint32 &p1,
                            const sint32 &p2_init, const uint8 *cost_init,
                            uint8 *cost_aggr, bool is_forward = true);

void CostAggregateUpDown(const uint8 *img_data, const sint32 &width,
                         const sint32 &height, const sint32 &min_disparity,
                         const sint32 &max_disparity, const sint32 &p1,
                         const sint32 &p2_init, const uint8 *cost_init,
                         uint8 *cost_aggr, bool is_forward = true);

void CostAggregateDagonal_1(const uint8 *img_data, const sint32 &width,
                            const sint32 &height, const sint32 &min_disparity,
                            const sint32 &max_disparity, const sint32 &p1,
                            const sint32 &p2_init, const uint8 *cost_init,
                            uint8 *cost_aggr, bool is_forward = true);

void CostAggregateDagonal_2(const uint8 *img_data, const sint32 &width,
                            const sint32 &height, const sint32 &min_disparity,
                            const sint32 &max_disparity, const sint32 &p1,
                            const sint32 &p2_init, const uint8 *cost_init,
                            uint8 *cost_aggr, bool is_forward = true);

void MedianFilter(const float32 *in, float32 *out, const sint32 &width,
                  const sint32 &height, const sint32 wnd_size);

void RemoveSpeckles(float32 *disparity_map, const sint32 &width,
                    const sint32 &height, const sint32 &diff_insame,
                    const uint32 &min_speckle_aera, const float32 &invalid_val);
} // namespace sgm_util