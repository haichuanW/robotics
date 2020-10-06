#include "../ADCensusBM/stereoprocessor.h"
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace cv;

constexpr auto Invalid_Float = std::numeric_limits<float>::infinity();

void ShowDisparityMap(float *disp_map, const int &width, const int &height,
                      const std::string &name);

int main(int argc, char *argv[]) {
  bool readSuccessfully = true;

  bool success = true;

  if (argc != 4) {
    std::cout << "Usage: " << std::endl;
    return -1;
  }

  YAML::Node config = YAML::LoadFile(argv[1]);

  uint dMin = config["dMin"].as<uint>();
  uint dMax = config["dMax"].as<uint>();
  uint censusWinH = config["censusWinH"].as<uint>();
  uint censusWinW = config["censusWinW"].as<uint>();
  Size censusWin = Size(censusWinH, censusWinW);
  float defaultBorderCost = config["defaultBorderCost"].as<float>();
  float lambdaAD = config["lambdaAD"].as<float>();
  float lambdaCensus = config["lambdaCensus"].as<float>();
  uint aggregatingIterations = config["aggregatingIterations"].as<uint>();
  uint colorThreshold1 = config["colorThreshold1"].as<uint>();
  uint colorThreshold2 = config["colorThreshold2"].as<uint>();
  uint maxLength1 = config["maxLength1"].as<uint>();
  uint maxLength2 = config["maxLength2"].as<uint>();
  uint colorDifference = config["colorDifference"].as<uint>();
  float pi1 = config["pi1"].as<float>();
  float pi2 = config["pi2"].as<float>();
  uint dispTolerance = config["dispTolerance"].as<uint>();
  uint votingThreshold = config["votingThreshold"].as<uint>();
  float votingRatioThreshold = config["votingRatioThreshold"].as<float>();
  uint maxSearchDepth = config["maxSearchDepth"].as<uint>();
  uint blurKernelSize = config["blurKernelSize"].as<uint>();
  uint cannyThreshold1 = config["cannyThreshold1"].as<uint>();
  uint cannyThreshold2 = config["cannyThreshold2"].as<uint>();
  uint cannyKernelSize = config["cannyKernelSize"].as<uint>();

  const cv::Mat img_left = cv::imread(argv[2]);
  const cv::Mat img_right = cv::imread(argv[3]);

  StereoProcessor sP(
      dMin, dMax, img_left, img_right, censusWin, defaultBorderCost, lambdaAD,
      lambdaCensus, aggregatingIterations, colorThreshold1, colorThreshold2,
      maxLength1, maxLength2, colorDifference, pi1, pi2, dispTolerance,
      votingThreshold, votingRatioThreshold, maxSearchDepth, blurKernelSize,
      cannyThreshold1, cannyThreshold2, cannyKernelSize);

  sP.init();
  sP.compute();

  const cv::Mat disparity = sP.getDisparity();

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
