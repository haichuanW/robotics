/***********************************************************************
 * Software License Agreement (BSD License)
 * 
 * Created on Sun April 19 2020
 * Basical function to create octree
 * 
 * author Haichuan Wang
 *************************************************************************/

#include <algorithm>
#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <random>
#include <chrono>

#include "KDTree.h"

template<typename coordinate_type, size_t dimensions>
std::ostream& operator<<(std::ostream& out, const point<coordinate_type, dimensions>& pt){
    out << '(';
    for (size_t i = 0; i < dimensions; ++i){
        if (i > 0)
            out << ", ";
        out << pt.get(i);
    }
    out << ')';
    return out;
}

int main(){
    typedef point<float,4> point4f;
    typedef kdtree<float,4> tree4f;

    std::FILE *pFile = fopen("000000.bin", "rb");
    fseek(pFile, 0, SEEK_END);    // file pointer goes to the end of the file
    long fileSize = ftell(pFile); // file size
    rewind(pFile);                // rewind file pointer to the beginning
    float *rawData = new float[fileSize];
    fread(rawData,sizeof(float),fileSize/sizeof(float),pFile);
    fclose(pFile);

    long number_of_points = fileSize / 4 / sizeof(float);

    std::vector<point4f> points;
    for(int i=0;i<number_of_points;++i){
        point4f point = {rawData[4*i],rawData[4*i+1],rawData[4*i+2],rawData[4*i+3]};
        points.push_back(point);
    }

    auto t1 = std::chrono::system_clock::now();
    
    tree4f tree1(points.begin(),points.end());

    //move constructor
    tree4f tree2=std::move(tree1);//change ownship from tree1 to tree2

    tree4f tree3 = tree2;

    tree4f tree4 = std::move(tree3);
    // copy constructor
    tree4f tree(tree4);

    point4f n = tree.knnSearch({ 9,2,1,0.1});
    
    auto t2 = std::chrono::system_clock::now();
    auto elapsed =std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1);

    std::cout << "knnSearch point: " << n << "\n";
    std::cout << "distance: " << tree.distance() << '\n';
    std::cout << "nodes visited: " << tree.visited() << '\n';

    std::cout << "time spent: " << elapsed.count() << "ms" << std::endl;

    delete []rawData;
    
    return 0;
}