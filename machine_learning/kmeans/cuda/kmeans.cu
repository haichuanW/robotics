#include "kmeans.h"


__device__ float square_l2_distance(float x1,float y1,float x2,float y2){
    return (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
}

// __restrict__ is pointer aliasing, where the same memory location can be accessed using different names 
__global__ void assign_clusters(const float* __restrict__ data_x,const float* __restrict__ data_y,int data_size,const float* __restrict__ means_x,const float* __restrict__ means_y,
                                float* __restrict__ new_sums_x,float* __restrict__ new_sums_y,int k,int* __restrict__ counts){
    const int index = blockIdx.x * blockDim.x + threadIdx.x;

    if(index>=data_size) return;

    float best_dist = FLT_MAX;
    int best_cluster = 0;
    for(int cluster=0;cluster<k;++cluster){
        const float dist = square_l2_distance(data_x[index],data_y[index],means_x[cluster],means_y[cluster]);
        if(dist<best_dist){
            best_dist = dist;
            best_cluster = cluster;
        }
    }
    //atomic operation (read-->modify-->overwrite)
    atomicAdd(&new_sums_x[best_cluster],data_x[index]);
    atomicAdd(&new_sums_y[best_cluster],data_y[index]);
    atomicAdd(&counts[best_cluster],1);
}

__global__ void compute_new_means(float* __restrict__ means_x,float* __restrict__ means_y,const float* __restrict__ new_sum_x,const float* __restrict__ new_sum_y,const int* __restrict__ counts) {
    const int cluster = threadIdx.x;
    const int count = max(1,counts[cluster]);
    means_x[cluster] = new_sum_x[cluster]/count;
    means_y[cluster] = new_sum_y[cluster]/count;
}

int main(int argc, const char* argv[]) {
    std::vector<float> h_x;
    std::vector<float> h_y;

    std::string line;
    std::ifstream infile("x.txt");
    if(infile.is_open()){
        while(std::getline(infile,line)){
            std::istringstream stream(line);
            float x1,y1;
            stream>>x1>>y1;
            h_x.push_back(x1);
            h_y.push_back(y1);
        }
    }

    infile.close();

    const size_t number_of_elements = h_x.size();

    Data d_data(number_of_elements, h_x, h_y);

    const size_t k =2, number_of_iterations = 50;

    // Random shuffle the data and pick the first
    // k points (i.e. k random points).
    std::random_device seed;
    std::mt19937 rng(seed());
    std::shuffle(h_x.begin(), h_x.end(), rng);
    std::shuffle(h_y.begin(), h_y.end(), rng);
    Data d_means(k, h_x, h_y);

    Data d_sums(k);

    int* d_counts;
    cudaMalloc(&d_counts, k * sizeof(int));
    cudaMemset(d_counts, 0, k * sizeof(int));

    const int threads = 1024;
    const int blocks = (number_of_elements + threads - 1) / threads;

    for (size_t iteration = 0; iteration < number_of_iterations; ++iteration) {
        cudaMemset(d_counts, 0, k * sizeof(int));
        d_sums.clear();

        assign_clusters<<<blocks, threads>>>(d_data.d_x,d_data.d_y,d_data.size,d_means.d_x,d_means.d_y,d_sums.d_x,d_sums.d_y,k,d_counts);
        cudaDeviceSynchronize();

        compute_new_means<<<1, k>>>(d_means.d_x,d_means.d_y,d_sums.d_x,d_sums.d_y,d_counts);
        cudaDeviceSynchronize();
    }

    std::vector<float> h_mean_x(k);
    std::vector<float> h_mean_y(k);
    cudaMemcpy(h_mean_x.data(),d_means.d_x,k*sizeof(float),cudaMemcpyDeviceToHost);
    cudaMemcpy(h_mean_y.data(),d_means.d_y,k*sizeof(float),cudaMemcpyDeviceToHost);

    for(int i=0;i<k;++i){
        std::cout << h_mean_x[i] << " " << h_mean_y[i] << std::endl;
    }

}