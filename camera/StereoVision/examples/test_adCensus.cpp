#include "../ADCensusStereo/ADCensusStereo.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std::chrono;

#include <opencv2/opencv.hpp>

void ShowDisparityMap(float *disp_map, const int &width, const int &height,
                      const std::string &name);

int main(int argv, char **argc) {
  if (argv < 3) {
    std::cout << "Usage: ./main filename1 filename2" << std::endl;
    return -1;
  }

  printf("Image Loading...");
  std::string path_left = argc[1];
  std::string path_right = argc[2];

  cv::Mat img_left = cv::imread(path_left, cv::IMREAD_COLOR);
  cv::Mat img_right = cv::imread(path_right, cv::IMREAD_COLOR);

  const int width = img_left.cols;
  const int height = img_right.rows;

  ADCensusOption ad_option;
  ad_option.min_disparity = 0;  // argv < 4 ? 0 : atoi(argc[3]);
  ad_option.max_disparity = 64; // argv < 5 ? 64 : atoi(argc[4]);
  ad_option.lrcheck_thres = 1.0f;

  ad_option.do_lr_check = true;

  ad_option.do_filling = false;

  ADCensusStereo ad_census;
  ad_census.Initialize(width, height, ad_option);

  cv::Mat disparity(cv::Size(width, height), CV_32F);
  ad_census.Match(img_left, img_right, disparity);

  ShowDisparityMap((float *)disparity.data, disparity.cols, disparity.rows,
                   "disp_left");

  return 0;
}

void ShowDisparityMap(float *disp_map, const int &width, const int &height,
                      const std::string &name) {
  const cv::Mat disp_mat = cv::Mat(height, width, CV_8UC1);
  float min_disp = float(width), max_disp = -float(width);
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      const float disp = abs(disp_map[i * width + j]);
      if (disp != Invalid_Float) {
        min_disp = std::min(min_disp, disp);
        max_disp = std::max(max_disp, disp);
      }
    }
  }
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      const float disp = abs(disp_map[i * width + j]);
      if (disp == Invalid_Float) {
        disp_mat.data[i * width + j] = 0;
      } else {
        disp_mat.data[i * width + j] =
            static_cast<uchar>((disp - min_disp) / (max_disp - min_disp) * 255);
      }
    }
  }

  cv::imshow(name, disp_mat);
  cv::Mat disp_color;
  applyColorMap(disp_mat, disp_color, cv::COLORMAP_JET);
  cv::imshow(name + "-color", disp_color);

  cv::imwrite("adcensus.png", disp_color);

  cv::waitKey(0);
}
