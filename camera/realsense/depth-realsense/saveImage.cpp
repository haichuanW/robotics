#include <yaml-cpp/yaml.h>
#include <cstring>
#include <fstream>
#include <iostream>              // for cout
#include <librealsense2/rs.hpp>  // Include RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void saveImage(std::string confileFile) {
  YAML::Node config = YAML::LoadFile(confileFile);

  int color_width = config["color_width"].as<int>();
  int color_height = config["color_height"].as<int>();
  int depth_width = config["depth_width"].as<int>();
  int depth_height = config["depth_height"].as<int>();

  int saveImage = config["saveImage"].as<int>();
  int showImage = config["showImage"].as<int>();

  float min_depth = config["min_depth"].as<float>();
  float laser_power = config["laser_power"].as<float>();

  std::string imgFolder = config["saveDir"].as<std::string>();

  std::cout << "color_width: " << color_width << std::endl;
  std::cout << "color_height: " << color_height << std::endl;
  std::cout << "depth_width: " << depth_width << std::endl;
  std::cout << "depth_height: " << depth_height << std::endl;
  std::cout << "saveImage: " << saveImage << std::endl;
  std::cout << "showImage: " << showImage << std::endl;
  std::cout << "min_depth: " << min_depth << std::endl;
  std::cout << "laser_power: " << laser_power << std::endl;

  std::cout << "save Image Folder: " << imgFolder << std::endl;

  // Contruct a pipeline which abstracts the device
  rs2::pipeline pipe;

  // Create a configuration for configuring the pipeline with a non default
  // profile
  rs2::config cfg;

  // Add desired streams to configuration
  cfg.enable_stream(RS2_STREAM_DEPTH, depth_width, depth_height, RS2_FORMAT_Z16,
                    30);
  cfg.enable_stream(RS2_STREAM_COLOR, color_width, color_height,
                    RS2_FORMAT_RGB8, 30);

  // Instruct pipeline to start streaming with the requested configuration
  rs2::pipeline_profile selection = pipe.start(cfg);

  rs2::device selected_device = selection.get_device();
  auto depth_sensor = selected_device.first<rs2::depth_sensor>();

  if (depth_sensor.supports(RS2_OPTION_MIN_DISTANCE)) {
    depth_sensor.set_option(RS2_OPTION_MIN_DISTANCE, min_depth);  //
  }

  if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
    depth_sensor.set_option(RS2_OPTION_LASER_POWER, laser_power);  //
  }

  rs2::align align_to(RS2_STREAM_COLOR);

  int index = 0;
  while (1) {
    // Camera warmup - dropping several first frames to let auto-exposure
    // stabilize
    rs2::frameset frames = pipe.wait_for_frames();
    rs2::frameset aligned_set = align_to.process(frames);
    // Get each frame
    rs2::frame depth_frame = aligned_set.get_depth_frame();
    rs2::frame color_frame = aligned_set.get_color_frame();

    // Creating OpenCV Matrix from a color image
    Mat depth(Size(color_width, color_height), CV_16UC1,
              (void *)depth_frame.get_data(), Mat::AUTO_STEP);
    Mat color(Size(color_width, color_height), CV_8UC3,
              (void *)color_frame.get_data(), Mat::AUTO_STEP);
    cv::cvtColor(color, color, cv::COLOR_BGR2RGB);

    if (saveImage == 1) {
      char rgb_name[128];
      std::string rgbdata_file = imgFolder + "/img_%05i.png";
      sprintf(rgb_name, rgbdata_file.c_str(), index);
      cv::imwrite(rgb_name, color);

      char depth_name[128];
      std::string depthdata_file = imgFolder + "/depth_%05i.png";
      sprintf(depth_name, depthdata_file.c_str(), index);
      cv::imwrite(depth_name, depth);
      index++;
    }

    if (showImage == 1) {
      cv::resize(depth, depth, cv::Size(600, 600));
      cv::resize(color, color, cv::Size(600, 600));

      imshow("Display Image", depth);
      imshow("Display Image1", color);
      waitKey(1);
    }
  }
}

int main(int argc, char *argv[]) {
  std::string configFile = argv[1];
  saveImage(configFile);
  return 0;
}
