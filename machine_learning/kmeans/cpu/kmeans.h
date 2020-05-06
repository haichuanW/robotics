#include "Point.h"

template<typename coordinate_type,size_t dimensions>
class KMeans{
public:
    typedef std::vector<point<coordinate_type,dimensions>> DataFrame;
    typedef point<coordinate_type,dimensions> point_type;
private:
    std::vector<point<coordinate_type,dimensions>> data;
    size_t k;
    size_t number_of_iteration;
public:
    template<typename iterator>
    KMeans(iterator start,iterator end,size_t k_,size_t number_of_iteration_);

    std::vector<point<coordinate_type,dimensions>> computeMean();
    
};

template<typename coordinate_type,size_t dimensions>
template<typename iterator>
KMeans<coordinate_type,dimensions>::KMeans(iterator start,iterator end,size_t k_,size_t number_of_iteration_){
    data.reserve(std::distance(start,end));
    for(auto i=start;i!=end;++i){
        data.emplace_back(*i);
    }

    k = k_;
    number_of_iteration = number_of_iteration_;
}

template<typename coordinate_type,size_t dimensions>
std::vector<point<coordinate_type,dimensions>> KMeans<coordinate_type,dimensions>::computeMean(){
    static std::random_device seed;
    static std::mt19937 random_number_generator(seed());
    std::uniform_int_distribution<size_t> indices(0, data.size() - 1);

    DataFrame means;
    for(size_t i=0;i<k;++i){
        means.push_back(data[indices(random_number_generator)]);
    }

    std::vector<size_t> labels(data.size());
    for(size_t iteration=0;iteration<number_of_iteration;++iteration){//number of iterations
        for(size_t point=0;point<data.size();++point){ //number of point
            double best_dist = std::numeric_limits<double>::max();
            size_t best_cluster = 0;
            for(size_t cluster=0;cluster<k;++cluster){
                const double dist = data[point].distance_l2(means[cluster]);
                if(dist<best_dist){
                    best_dist = dist;
                    best_cluster = cluster;
                }
            }
            labels[point] = best_cluster;
        }
    }

    // update mean value
    std::vector<size_t> counts(k,0);
    for(size_t point=0;point<data.size();++point){
        const auto cluster = labels[point];
        means[cluster] += data[point];
        counts[cluster]+=1;
    }

    for(size_t cluster=0;cluster<k;++cluster){
        const auto count = std::max<coordinate_type>(1,counts[cluster]);
        means[cluster] = means[cluster]/count;
    }

    return means;
}




