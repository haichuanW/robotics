#include <cuda_runtime.h>
#include <algorithm>
#include <cfloat>
#include <chrono>
#include <random>
#include <vector>
#include <iostream>
#include <device_launch_parameters.h>
#include <fstream>
#include <sstream>

using namespace std;

struct Point{
    float x,y;
};

struct Data{
    int size{0},bytes{0};
    float *d_x{nullptr};
    float *d_y{nullptr};
    explicit Data(int size):size(size),bytes(size*sizeof(float)){
        cudaMalloc(&d_x,bytes);
        cudaMalloc(&d_y,bytes);
    }

    Data(int size,std::vector<float>& h_x,std::vector<float>& h_y):size(size),bytes(size*sizeof(float)){
        cudaMalloc(&d_x,bytes);
        cudaMalloc(&d_y,bytes);
        cudaMemcpy(d_x,h_x.data(),bytes,cudaMemcpyHostToDevice);
        cudaMemcpy(d_y,h_y.data(),bytes,cudaMemcpyHostToDevice);
    }

    ~Data(){
        cudaFree(d_x);
        cudaFree(d_y);
    }

    void clear() {
        cudaMemset(d_x, 0, bytes);
        cudaMemset(d_y, 0, bytes);
    }
};



