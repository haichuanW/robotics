/***********************************************************************
 * Software License Agreement (BSD License)
 * 
 * Created on Sun April 19 2020
 * Basical function to create octree
 * 
 * author Haichuan Wang
 *************************************************************************/

#ifndef _POINT_H_
#define _POINT_H_

#include <iostream>
#include <algorithm>
#include <limits>
#include <vector>
#include <random>
#include <array>



template<typename coordinate_type, size_t dimensions>
class point{
public:
    point(std::array<coordinate_type, dimensions> c) : coords_(c){}
    
    point(std::initializer_list<coordinate_type> list){
        size_t n = std::min(dimensions, list.size());
        std::copy_n(list.begin(), n, coords_.begin());
    }
    
    // return point value by index
    coordinate_type get(size_t index) const{
        return coords_[index];
    }
    
    coordinate_type operator[] (size_t index) const{
        return get(index);
    }

    point<coordinate_type,dimensions>& operator+=(const point& pt){
        for(size_t i=0;i<dimensions;++i){
            this->coords_[i] += pt[i];
        }
        return *this;
    }

    point<coordinate_type,dimensions>& operator/(const coordinate_type& r){
        for(size_t i=0;i<dimensions;++i){
            coords_[i] /= r;
        }
        return *this;
    }

    // return distance between two points
    double distance_l2(const point& pt) const
    {
        double dist = 0;
        for (size_t i = 0; i < dimensions; ++i){
            double d = get(i) - pt.get(i);
            dist += d * d;
        }
        return sqrt(dist);
    }

    double distance_l1(const point& pt) const{
        double dist = 0;
        for(size_t i=0;i<dimensions;++i){
            double d = get(i) - pt.get(i);
            dist += abs(d);
        }
        return dist;
    }
private:
    std::array<coordinate_type, dimensions> coords_;
};

#endif