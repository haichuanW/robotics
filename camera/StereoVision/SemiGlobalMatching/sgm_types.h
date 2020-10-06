#ifndef SGM_TYPES_H_
#define SGM_TYPES_H_

#include <cstdint>
#include <limits>
#include <vector>
using std::pair;
using std::vector;
using namespace std;

#ifndef SAFE_DELETE
#define SAFE_DELETE(P)                                                         \
  {                                                                            \
    if (P)                                                                     \
      delete[](P);                                                             \
    (P) = nullptr;                                                             \
  }
#endif

typedef uint8_t uint8;
typedef uint8_t uint8;   // 无符号8位整数
typedef int16_t sint16;  // 有符号16位整数
typedef uint16_t uint16; // 无符号16位整数
typedef int32_t sint32;  // 有符号32位整数
typedef uint32_t uint32; // 无符号32位整数
typedef int64_t sint64;  // 有符号64位整数
typedef uint64_t uint64; // 无符号64位整数
typedef float float32;   // 单精度浮点
typedef double float64;  // 双精度浮点

constexpr auto Invalid_Float = std::numeric_limits<float>::infinity();

constexpr auto Large_Float = 99999.0f;
constexpr auto Small_Float = -99999.0f;

#endif
