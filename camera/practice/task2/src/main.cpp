#include "guidedfilter.h"
#include <iostream>
#include <opencv2/opencv.hpp>

float distance(int x, int y, int i, int j) { return float(sqrt(pow(x - i, 2) + pow(y - j, 2))); }

double gaussian(float x, double sigma) { return exp(-(pow(x, 2)) / (2 * pow(sigma, 2))) / (2 * CV_PI * pow(sigma, 2)); }

void applyBilateralFilter(cv::Mat source, cv::Mat filteredImage, int x, int y, int diameter, double sigmaI,
                          double sigmaS) {
    double iFiltered = 0;
    double wP = 0;
    int neighbor_x = 0;
    int neighbor_y = 0;
    int half = diameter / 2;

    for (int i = 0; i < diameter; i++) {
        for (int j = 0; j < diameter; j++) {
            neighbor_x = x - (half - i);
            neighbor_y = y - (half - j);
            double gi = gaussian(source.at<uchar>(neighbor_x, neighbor_y) - source.at<uchar>(x, y), sigmaI);
            double gs = gaussian(distance(x, y, neighbor_x, neighbor_y), sigmaS);
            double w = gi * gs;
            iFiltered = iFiltered + source.at<uchar>(neighbor_x, neighbor_y) * w;
            wP = wP + w;
        }
    }
    iFiltered = iFiltered / wP;
    filteredImage.at<double>(x, y) = iFiltered;
}

cv::Mat OurBilateralFilter(cv::Mat source, int diameter, double sigmaI, double sigmaS) {
    cv::Mat filteredImage = cv::Mat::zeros(source.rows, source.cols, CV_64F);
    int width = source.cols;
    int height = source.rows;

    for (int i = 2; i < height - 2; i++) {
        for (int j = 2; j < width - 2; j++) {
            applyBilateralFilter(source, filteredImage, i, j, diameter, sigmaI, sigmaS);
        }
    }
    cv::Mat res;
    filteredImage.convertTo(res, CV_8UC1);
    return res;
}

void OurFilterGaussian(const cv::Mat &input, cv::Mat &output, const cv::Size ksize, double sigmaX, double sigmaY) {
    const int ksize_wd = ksize.width, ksize_ht = ksize.height;
    const int width = input.cols, height = input.rows;

    output = cv::Mat::zeros(cv::Size(width, height), CV_8UC1);

    double total = 0.0;
    for (int i = -ksize_ht / 2; i <= ksize_ht / 2; ++i) {
        for (int j = -ksize_wd / 2; j <= ksize_wd / 2; ++j) {
            total += exp(-(i * i + j * j) / (2 * sigmaX * sigmaY)) / sqrt(2 * M_PI * sigmaX * sigmaY);
        }
    }

    for (int r = ksize_ht / 2; r < height - ksize_ht / 2; ++r) {
        for (int c = ksize_wd / 2; c < width - ksize_wd / 2; ++c) {

            // box filter
            double sum = 0;
            for (int i = -ksize_ht / 2; i <= ksize_ht / 2; ++i) {
                for (int j = -ksize_wd / 2; j <= ksize_wd / 2; ++j) {
                    sum += static_cast<double>(input.at<uchar>(r + i, c + j)) *
                           exp(-(i * i + j * j) / (2 * sigmaX * sigmaY)) / sqrt(2 * M_PI * sigmaX * sigmaY);
                }
            }
            output.at<uchar>(r, c) = static_cast<uchar>(sum / total);
        }
    }
}

double SSD(const cv::Mat &image1, const cv::Mat &image2) {
    CV_Assert(image1.size() == image2.size());
    CV_Assert(image1.type() == image2.type());

    double res = 0;
    for (int i = 0; i < image1.rows * image1.cols; ++i) {
        res += pow(static_cast<double>(image1.data[i]) - static_cast<double>(image2.data[i]), 2);
    }
    return res / (image1.rows * image1.cols);
}

double RMSE(const cv::Mat &image1, const cv::Mat &image2) {
    CV_Assert(image1.size() == image2.size());
    CV_Assert(image1.type() == image2.type());

    double res = 0;
    for (int i = 0; i < image1.rows * image1.cols; ++i) {
        res += pow(static_cast<double>(image1.data[i]) - static_cast<double>(image2.data[i]), 2);
    }
    return sqrt(res / (image1.rows * image1.cols));
}

double PSNR(const cv::Mat &image1, const cv::Mat &image2) {
    CV_Assert(image1.size() == image2.size());
    CV_Assert(image1.type() == image2.type());

    double mse = SSD(image1, image2);
    double pixel_max = 255.0;
    return 20.0 * log10(pixel_max / sqrt(mse));
}

int main(int argc, char **argv) {
    cv::Mat im = cv::imread(argv[1], 0);
    cv::Mat im1 = cv::imread(argv[2], 0);

    if (im.data == nullptr) {
        std::cerr << "Failed to load image" << std::endl;
    }

    // cv::imshow("im", im);
    // cv::waitKey();

    cv::Mat noise(im.size(), im.type());
    uchar mean = 0;
    uchar stddev = 25;
    cv::randn(noise, mean, stddev);

    im += noise;

    // our Gaussian Filter
    // cv::Mat output1;
    // OurFilterGaussian(im, output1, cv::Size(7, 7), 1, 1);
    // cv::imshow("OurFiler", output1);
    // cv::waitKey(0);

    // save result to disk
    // const int d = 5;
    // double sigmaColor = 12.0, sigmaSpace = 16.0;
    // for (int i = 1; i <= 4; ++i) {
    //     for (int j = 1; j <= 4; ++j) {
    //         sigmaColor = i * 6.0;
    //         sigmaSpace = j * 8.0;
    //         cv::Mat filteredImageOwn = OurBilateralFilter(im, d, sigmaColor, sigmaSpace);
    //         cv::imwrite("../data/resultColor" + std::to_string(i * 6) + "Space" + std::to_string(j * 8) + ".png",
    //                     filteredImageOwn);
    //     }
    // }
    // cv::Mat bilteralRes = OurBilateralFilter(im, d, sigmaColor, sigmaSpace);

    int r = 4;
    double eps = 0.2 * 0.2 * 255 * 255;

    cv::Mat guide = guidedFilter(im, im1, r, eps);

    cv::imshow("OurFiler", guide);

    cv::imwrite("../data/guide.png", guide);
    cv::waitKey(0);

    // HW (1 point for each metric, max 5 points):
    // - compare your images
    //		- SSD
    //		- RMSE (Root Mean Squared Error)
    //		- PSNR ..
    //	  - ....

    return 0;
}