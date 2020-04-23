/***********************************************************************
 * Software License Agreement (BSD License)
 * 
 * Created on Sun April 19 2020
 * Basical function to create octree
 * 
 * author Haichuan Wang
 *************************************************************************/

#include <iostream>

template<typename coordinate_type, size_t dimensions>
class point{
public:
    point(std::array<coordinate_type, dimensions> c) : coords_(c){}
    
    point(std::initializer_list<coordinate_type> list){
        size_t n = std::min(dimensions, list.size());
        std::copy_n(list.begin(), n, coords_.begin());
    }
    
    // return point value by index
    coordinate_type get(size_t index) const
    {
        return coords_[index];
    }
    
    // return distance between two points
    double distance(const point& pt) const
    {
        double dist = 0;
        for (size_t i = 0; i < dimensions; ++i)
        {
            double d = get(i) - pt.get(i);
            dist += d * d;
        }
        return dist;
    }
private:
    std::array<coordinate_type, dimensions> coords_;
};