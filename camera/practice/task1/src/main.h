#pragma once

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>

void StereoEstimation_Naive(const int &window_size, const int &dmin, int height, int width, cv::Mat &image1,
                            cv::Mat &image2, cv::Mat &naive_disparities, const double &scale);

void Disparity2PointCloud(const std::string &output_file, int height, int width, cv::Mat &disparities,
                          const int &window_size, const int &dmin, const double &baseline, const double &focal_length);

void StereoEstimation_DP(const cv::Mat &image1, const cv::Mat &image2, cv::Mat &disp, int occusionVal);

void showPointCloud(const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &pointcloud);

void showPointCloudPangolin(const cv::Mat &left, const cv::Mat &disparity, const double &baseline,
                            const double &focal_length, const int &cx = 300, const int &cy = 200);