#include "main.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>

int main(int argc, char **argv) {

    ////////////////
    // Parameters //
    ////////////////

    // camera setup parameters
    const double focal_length = 1247;
    const double baseline = 213;

    // stereo estimation parameters
    const int dmin = 67;
    const int window_size = 3;
    const double weight = 500;
    const double scale = 3;

    // stereo estimate parameters(dynamic programming)
    const int occCost = 10;

    ///////////////////////////
    // Commandline arguments //
    ///////////////////////////

    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " IMAGE1 IMAGE2 OUTPUT_FILE" << std::endl;
        return 1;
    }

    cv::Mat image1 = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat image2 = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
    const std::string output_file = argv[3];

    if (!image1.data) {
        std::cerr << "No image1 data" << std::endl;
        return EXIT_FAILURE;
    }

    if (!image2.data) {
        std::cerr << "No image2 data" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "------------------ Parameters -------------------" << std::endl;
    std::cout << "focal_length = " << focal_length << std::endl;
    std::cout << "baseline = " << baseline << std::endl;
    std::cout << "window_size = " << window_size << std::endl;
    std::cout << "occlusion weights = " << weight << std::endl;
    std::cout << "disparity added due to image cropping = " << dmin << std::endl;
    std::cout << "scaling of disparity images to show = " << scale << std::endl;
    std::cout << "output filename = " << argv[3] << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;

    int height = image1.size().height;
    int width = image1.size().width;

    ////////////////////
    // Reconstruction //
    ////////////////////

    // Naive disparity image
    // cv::Mat naive_disparities = cv::Mat::zeros(height - window_size, width -
    // window_size, CV_8UC1);
    cv::Mat naive_disparities = cv::Mat::zeros(height, width, CV_8UC1);

    // StereoEstimation_Naive(window_size, dmin, height, width, image1, image2, naive_disparities, scale);

    cv::Mat dp_disparities = cv::Mat::zeros(height, width, CV_8UC1);

    StereoEstimation_DP(image1, image2, dp_disparities, occCost);
    ////////////
    // Output //
    ////////////

    // reconstruction
    // Disparity2PointCloud(output_file, height, width, dp_disparities, window_size, dmin, baseline, focal_length);

    // show pointcloud with pangolin

    showPointCloudPangolin(image1, dp_disparities, baseline, focal_length);

    // // save / display images
    // std::stringstream out1;
    // out1 << output_file << "_naive.png";
    // cv::imwrite(out1.str(), naive_disparities);

    // cv::namedWindow("Naive", cv::WINDOW_AUTOSIZE);
    // cv::imshow("Naive", naive_disparities);

    // cv::namedWindow("dynamic programming", cv::WINDOW_AUTOSIZE);
    // cv::imshow("dynamic programming", dp_disparities);

    // cv::imwrite("../data/naive_disparities.png", naive_disparities);
    // cv::imwrite("../data/dp_disparities.png", dp_disparities);

    cv::waitKey(0);

    return 0;
}

void StereoEstimation_DP(const cv::Mat &image1, const cv::Mat &image2, cv::Mat &disp, int occCost) {
    const int width = image1.cols, height = image1.rows;

    cv::Mat costMap(width, width, CV_32SC1);
    cv::Mat directionMap(width, width, CV_32SC1);

    for (int i = 0; i < height; ++i) {
        costMap = cv::Mat::zeros(width, width, CV_32SC1);
        directionMap = cv::Mat::zeros(width, width, CV_32SC1);
        for (int j = 0; j < width; ++j) {
            costMap.at<int>(0, j) = j * occCost;
            costMap.at<int>(j, 0) = j * occCost;
        }

        for (int j = 1; j < width; ++j) {
            for (int k = 1; k < width; ++k) {
                int min1 = costMap.at<int>(j - 1, k - 1) +
                           abs(static_cast<int>(image1.at<uchar>(i, j)) - static_cast<int>(image2.at<uchar>(i, k)));
                int min2 = costMap.at<int>(j - 1, k) + occCost;
                int min3 = costMap.at<int>(j, k - 1) + occCost;

                // int minVal = std::min(std::min(min1, min2), min3);

                // costMap.at<int>(j, k) = minVal;
                if (min1 <= min2 && min1 <= min3) {
                    costMap.at<int>(j, k) = min1;
                    directionMap.at<int>(j, k) = 1;
                } else if (min2 <= min3 && min2 < min1) {
                    costMap.at<int>(j, k) = min2;

                    directionMap.at<int>(j, k) = 2;
                } else {
                    costMap.at<int>(j, k) = min3;
                    directionMap.at<int>(j, k) = 3;
                }
            }
        }

        int P = width - 1, Q = width - 1;
        while (P > 0 && Q > 0) {
            switch (directionMap.at<int>(P, Q)) {
            case 1:
                disp.at<uchar>(i, P) = abs(P - Q);
                P--;
                Q--;
                break;
            case 2:
                disp.at<uchar>(i, P) = 0;
                P--;
                break;
            default:
                disp.at<uchar>(i, Q) = 0;
                Q--;
                break;
            }
        }
    }
}

void StereoEstimation_Naive(const int &window_size, const int &dmin, int height, int width, cv::Mat &image1,
                            cv::Mat &image2, cv::Mat &naive_disparities, const double &scale) {
    int half_window_size = window_size / 2;

    for (int i = half_window_size; i < height - half_window_size; ++i) {

        std::cout << "Calculating disparities for the naive approach... "
                  << std::ceil(((i - half_window_size + 1) / static_cast<double>(height - window_size + 1)) * 100)
                  << "%\r" << std::flush;

        for (int j = half_window_size; j < width - half_window_size; ++j) {
            int min_ssd = INT_MAX;
            int disparity = 0;

            for (int d = -j + half_window_size; d < width - j - half_window_size; ++d) {
                int ssd = 0;

                // TODO: sum up matching cost (ssd) in a window NOLINT
                for (int lr = i - half_window_size; lr <= i + half_window_size; ++lr) {
                    for (int lc = j - half_window_size; lc <= j + half_window_size; ++lc) {
                        ssd += pow(((int)image1.at<uchar>(lr, lc) - (int)image2.at<uchar>(lr, lc + d)), 2);
                    }
                }

                if (ssd < min_ssd) {
                    min_ssd = ssd;
                    disparity = d;
                }
            }

            naive_disparities.at<uchar>(i - half_window_size, j - half_window_size) = std::abs(disparity) * scale;
        }
    }

    std::cout << "Calculating disparities for the naive approach... Done.\r" << std::flush;
    std::cout << std::endl;
}

void Disparity2PointCloud(const std::string &output_file, int height, int width, cv::Mat &disparities,
                          const int &window_size, const int &dmin, const double &baseline, const double &focal_length) {
    std::stringstream out3d;
    out3d << output_file << ".xyz";
    std::ofstream outfile(out3d.str());
    for (int i = 0; i < height - window_size; ++i) {
        std::cout << "Reconstructing 3D point cloud from disparities... "
                  << std::ceil(((i) / static_cast<double>(height - window_size + 1)) * 100) << "%\r" << std::flush;
        for (int j = 0; j < width - window_size; ++j) {
            if (disparities.at<uchar>(i, j) == 0)
                continue;

            // TODO
            const double Z = focal_length * baseline / (double)disparities.at<uchar>(i, j) + dmin;
            const double X = Z * j / focal_length;
            const double Y = Z * i / focal_length;
            //
            outfile << X << " " << Y << " " << Z << std::endl;
        }
    }

    std::cout << "Reconstructing 3D point cloud from disparities... Done.\r" << std::flush;
    std::cout << std::endl;
}

void showPointCloudPangolin(const cv::Mat &left, const cv::Mat &disparity, const double &baseline,
                            const double &focal_length, const int &cx, const int &cy) {
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> pointcloud;
    cv::Mat disp;

    disparity.convertTo(disp, CV_32F, 1.0 / 16.0f);

    for (int v = 0; v < left.rows; v++)
        for (int u = 0; u < left.cols; u++) {

            if (disp.at<float>(v, u) <= 0.0 || disp.at<float>(v, u) >= 96.0)
                continue;
            Eigen::Vector4d point(0, 0, 0, left.at<uchar>(v, u) / 255.0);

            double depth = focal_length * baseline / (1000.0 * (double)disp.at<float>(v, u));
            point[0] = (u - cx) * depth / focal_length;
            point[1] = (v - cy) * depth / focal_length;
            point[2] = depth;

            pointcloud.push_back(point);
        }

    showPointCloud(pointcloud);
}

void showPointCloud(const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &pointcloud) {

    if (pointcloud.empty()) {
        std::cerr << "Point cloud is empty!" << std::endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
                                      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                                .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p : pointcloud) {
            glColor3f(p[3], p[3], p[3]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        // std::usleep(5000); // sleep 5 ms
        // cv::waitKey(0);
    }
    return;
}
