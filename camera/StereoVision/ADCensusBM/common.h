#ifndef COMMON_H
#define COMMON_H
#include <chrono>

#define COST_TYPE_NOT_FLOAT 1

#if COST_TYPE_NOT_FLOAT
const int COST_FACTOR = 65000;
typedef unsigned short int costType;
#define COST_MAP_TYPE CV_16U
#else
const float COST_FACTOR = 1.0;
typedef float costType;
#define COST_MAP_TYPE CV_32F
#endif

class TicToc {
public:
  TicToc() { tic(); }

  void tic() { start = std::chrono::system_clock::now(); }

  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};

#endif // COMMON_H
