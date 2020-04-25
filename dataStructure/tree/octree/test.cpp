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

#include "octree.h"

typedef Vec3<float> Vec3f;

int main(){

    std::FILE *pFile = fopen("000000.bin", "rb");
    fseek(pFile, 0, SEEK_END);    // file pointer goes to the end of the file
    long fileSize = ftell(pFile); // file size
    rewind(pFile);                // rewind file pointer to the beginning
    float *rawData = new float[fileSize];
    fread(rawData,sizeof(float),fileSize/sizeof(float),pFile);
    fclose(pFile);

    long number_of_points = fileSize / 4 / sizeof(float);

    std::vector<Vec3f> points;
    for(int i=0;i<number_of_points;++i){
        points.push_back(Vec3f(rawData[4*i],rawData[4*i+1],rawData[4*i+2]));
    }

    delete[] rawData;
    auto t1 = std::chrono::system_clock::now();

    Octree<float>* octree = new Octree<float>(Vec3<float>(-0.5 , -5.5 , -4.35),Vec3<float>(78.5 , 50.5 ,  7.25));

    OctreePoint<float> *octreePoints = new OctreePoint<float>[number_of_points];
    for(int i=0;i<number_of_points;++i){
        octreePoints[i].setPosition(points[i]);
        octree->insert(octreePoints + i);
    }

    double best_dist = 0;
    octree->knnSearch(Vec3<float>(9.0,19.0,0.6),best_dist);
    
    delete[] octreePoints;
    delete octree;

    auto t2 = std::chrono::system_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1);
    std::cout << duration.count() << std::endl;

    return 0;
}