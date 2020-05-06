#include <iostream>

#include "kmeans.h"

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

    typedef KMeans<float,4> kmeans4f;
    typedef point<float,4> point4f;

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

    point4f pt = {1.1,1,2,3};
    kmeans4f kmeans(points.begin(),points.end(),3,3000);

    auto res = kmeans.computeMean();

    for(size_t i=0;i<res.size();++i){
        point4f p = res[i];
        std::cout << p << std::endl;
    }


    return 0;
}