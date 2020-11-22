#include "Eigen/Dense"
#include "icp.h"
#include <fstream>
#include <iostream>
#include <numeric>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <random>
#include <sstream>
#include <vector>
Eigen::MatrixXd readPlyFile(const std::string &filename) {
    std::ifstream input1(filename);

    std::string line1;

    for (int i = 0; i < 3; ++i) {
        std::getline(input1, line1);
    }

    int numLine;
    std::string s1, s2;
    std::getline(input1, line1);

    std::istringstream iss(line1);
    iss >> s1 >> s2 >> numLine;

    for (int i = 0; i < 9; ++i) {
        std::getline(input1, line1);
    }

    Eigen::MatrixXd point(numLine, 6);

    for (int i = 0; i < numLine; ++i) {
        std::getline(input1, line1);
        std::istringstream iss1(line1);
        iss1 >> point(i, 0) >> point(i, 1) >> point(i, 2) >> point(i, 3) >> point(i, 4) >>
            point(i, 5);
    }

    return point;

    // std::cout << "num of Lines: " << numLine << std::endl;
}

static double randomUniform(double low, double high) {
    static std::mt19937 mt;
    std::uniform_real_distribution<double> dis(low, high);
    return dis(mt);
}

void addNoise(const Eigen::MatrixXd &A, Eigen::MatrixXd &res) {
    for (int i = 0; i < A.rows(); ++i) {
        for (int j = 0; j < 3; ++j) {
            res(i, j) = A(i, j) + randomUniform(0, 1);
        }
    }
}

void visualPointCloud(const Eigen::MatrixXd &A, std::uint8_t r = 255, std::uint8_t g = 15,
                      std::uint8_t b = 15) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    // std::uint8_t r(255), g(15), b(15);

    for (int i = 0; i < A.rows(); ++i) {
        pcl::PointXYZRGB basic_point;
        basic_point.x = A(i, 0);
        basic_point.y = A(i, 1);
        basic_point.z = A(i, 2);

        std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 |
                             static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
        basic_point.rgb = *reinterpret_cast<float *>(&rgb);

        basic_cloud_ptr->points.push_back(basic_point);
    }

    basic_cloud_ptr->width = basic_cloud_ptr->size();
    basic_cloud_ptr->height = 1;

    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZRGB>(basic_cloud_ptr, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
                                             "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

void visualPointCloud(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::uint8_t r(255), g(15), b(15);
    for (int i = 0; i < A.rows(); ++i) {
        pcl::PointXYZRGB basic_point;
        basic_point.x = A(i, 0);
        basic_point.y = A(i, 1);
        basic_point.z = A(i, 2);

        std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 |
                             static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
        basic_point.rgb = *reinterpret_cast<float *>(&rgb);

        basic_cloud_ptr->points.push_back(basic_point);
    }

    r = 15;
    b = 255;
    for (int i = 0; i < B.rows(); ++i) {
        pcl::PointXYZRGB basic_point;
        basic_point.x = B(i, 0);
        basic_point.y = B(i, 1);
        basic_point.z = B(i, 2);

        std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 |
                             static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
        basic_point.rgb = *reinterpret_cast<float *>(&rgb);

        basic_cloud_ptr->points.push_back(basic_point);
    }

    basic_cloud_ptr->width = A.rows() + B.rows();  // basic_cloud_ptr->size();
    basic_cloud_ptr->height = 1;

    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZRGB>(basic_cloud_ptr, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
                                             "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

int main(int argc, char *argv[]) {
    if (argc == 1) {
        return -1;
    }

    Eigen::MatrixXd A = readPlyFile(argv[1]);
    // Eigen::MatrixXd B = readPlyFile(argv[2]);

    // visualPointCloud(A, 200);

    // // int row = std::min(A.rows(), B.rows());
    int row = A.rows();
    Eigen::MatrixXd B(A.rows(), A.cols());

    addNoise(A, B);

    ICP myIcp(30);
    myIcp.run_scan_matcher(A.block(0, 0, row, 3), B.block(0, 0, row, 3));

    Eigen::MatrixXd T = myIcp.get_transformation();

    Eigen::MatrixXd res = A * T.block(0, 0, 3, 3);

    for (int i = 0; i < res.rows(); ++i) {
        res.row(i) = res.row(i) + T.block(0, 3, 3, 1).transpose();
    }

    visualPointCloud(B, res);

    std::cout << "transform matrix:\n " << T << std::endl;

    return 0;
}
