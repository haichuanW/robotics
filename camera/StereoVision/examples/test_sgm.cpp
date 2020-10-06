
#include "../SemiGlobalMatching/SemiGlobalMatching.h"
#include "fbs_filter.h"
#include <chrono>
using namespace std::chrono;

// opencv library
#include <opencv2/opencv.hpp>

int main(int argv, char **argc) {
  if (argv < 3) {
    std::cout << "Usage: " << std::endl;
    return -1;
  }

  std::string path_left = argc[1];
  std::string path_right = argc[2];

  cv::Mat img_left_c = cv::imread(path_left, cv::IMREAD_COLOR);
  cv::Mat img_left = cv::imread(path_left, cv::IMREAD_GRAYSCALE);
  cv::Mat img_right = cv::imread(path_right, cv::IMREAD_GRAYSCALE);

  if (img_left.data == nullptr || img_right.data == nullptr) {
    std::cout << "" << std::endl;
    return -1;
  }
  if (img_left.rows != img_right.rows || img_left.cols != img_right.cols) {
    std::cout << "" << std::endl;
    return -1;
  }

  const sint32 width = static_cast<uint32>(img_left.cols);
  const sint32 height = static_cast<uint32>(img_right.rows);

  auto bytes_left = new uint8[width * height];
  auto bytes_right = new uint8[width * height];
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      bytes_left[i * width + j] = img_left.at<uint8>(i, j);
      bytes_right[i * width + j] = img_right.at<uint8>(i, j);
    }
  }

  printf("Loading Views...Done!\n");

  SemiGlobalMatching::SGMOption sgm_option;
  sgm_option.num_paths = 8;
  sgm_option.min_disparity = argv < 4 ? 0 : atoi(argc[3]);
  sgm_option.max_disparity = argv < 5 ? 64 : atoi(argc[4]);
  sgm_option.census_size = SemiGlobalMatching::Census5x5;
  sgm_option.is_check_lr = true;
  sgm_option.lrcheck_thres = 1.0f;
  sgm_option.is_check_unique = true;
  sgm_option.uniqueness_ratio = 0.99;
  sgm_option.is_remove_speckles = true;
  sgm_option.min_speckle_aera = 50;
  sgm_option.p1 = 10;
  sgm_option.p2_init = 150;
  sgm_option.is_fill_holes = false;

  printf("w = %d, h = %d, d = [%d,%d]\n\n", width, height,
         sgm_option.min_disparity, sgm_option.max_disparity);

  SemiGlobalMatching sgm;

  printf("SGM Initializing...\n");
  auto start = std::chrono::steady_clock::now();
  if (!sgm.Initialize(width, height, sgm_option)) {
    std::cout << "SGM initizlize falied" << std::endl;
    return -2;
  }
  auto end = std::chrono::steady_clock::now();
  auto tt = duration_cast<std::chrono::milliseconds>(end - start);
  printf("SGM Initializing Done! Timing : %lf s\n\n", tt.count() / 1000.0);

  printf("SGM Matching...\n");
  start = std::chrono::steady_clock::now();
  auto disparity = new float32[uint32(width * height)]();
  if (!sgm.Match(bytes_left, bytes_right, disparity)) {
    std::cout << "SGM match failed" << std::endl;
    return -2;
  }
  end = std::chrono::steady_clock::now();
  tt = duration_cast<std::chrono::milliseconds>(end - start);
  printf("\nSGM Matching...Done! Timing :   %lf s\n", tt.count() / 1000.0);

  cv::Mat disp_mat = cv::Mat(height, width, CV_8UC1);
  float min_disp = width, max_disp = -width;
  for (sint32 i = 0; i < height; i++) {
    for (sint32 j = 0; j < width; j++) {
      const float32 disp = disparity[i * width + j];
      if (disp != Invalid_Float) {
        min_disp = std::min(min_disp, disp);
        max_disp = std::max(max_disp, disp);
      }
    }
  }
  for (sint32 i = 0; i < height; i++) {
    for (sint32 j = 0; j < width; j++) {
      const float32 disp = disparity[i * width + j];
      if (disp == Invalid_Float) {
        disp_mat.data[i * width + j] = 0;
      } else {
        disp_mat.data[i * width + j] =
            static_cast<uchar>((disp - min_disp) / (max_disp - min_disp) * 255);
      }
    }
  }

  // implement bilteral solver
  cv::Mat confidence(disp_mat.size(), CV_8UC1, cv::Scalar(0));
  // float fb = 50;
  uchar *ptr_disp = (uchar *)disp_mat.data;
  uchar *ptr_conf = (uchar *)confidence.data;
  for (int i = 0; i < disp_mat.rows; i++) {
    for (int j = 0; j < disp_mat.cols; j++) {
      *ptr_conf++ = *ptr_disp++ > 0 ? 255 : 0;
    }
  }

  cv::Mat smooth_disp;
  cv::ximgproc::fastBilateralSolverFilter(img_left, disp_mat, confidence,
                                          smooth_disp, 8, 8, 8, 50);

  cv::imshow("disp", disp_mat);
  cv::Mat disp_color;
  applyColorMap(disp_mat, disp_color, cv::COLORMAP_JET);
  cv::imshow("color", disp_color);

  applyColorMap(smooth_disp, disp_color, cv::COLORMAP_JET);
  cv::imshow("smooth_color", disp_color);

  // std::string disp_map_path = argc[1];
  // disp_map_path += ".d.png";
  // std::string disp_color_map_path = argc[1];
  // disp_color_map_path += ".c.png";
  // cv::imwrite(disp_map_path, disp_mat);
  // cv::imwrite(disp_color_map_path, disp_color);

  cv::waitKey(0);

  delete[] disparity;
  disparity = nullptr;
  delete[] bytes_left;
  bytes_left = nullptr;
  delete[] bytes_right;
  bytes_right = nullptr;

  return 0;
}
