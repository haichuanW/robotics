#ifndef detector_hpp
#define detector_hpp

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>


// Harris corner detection
void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img){
    // Detector parameters
    int blockSize = 2;     // A blockSize neighborhood is considered
    int apertureSize = 3;  // Aperture parameter for the Sobel operator
    assert(1 == apertureSize % 2);  // Aperture size must be odd
    int minResponse = 100; // Minimum value for a corner in the scaled (0...255) response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    // Non-maximum suppression (NMS) settings
    double maxOverlap = 0.0;  // Maximum overlap between two features in %

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

     // Apply non-maximum suppression (NMS)
    for (size_t j = 0; j < dst_norm.rows; j++) {
        for (size_t i = 0; i < dst_norm.cols; i++) {
            int response = (int)dst_norm.at<float>(j, i);

            // Apply the minimum threshold for Harris cornerness response
            if (response < minResponse) continue;

            // Otherwise create a tentative new keypoint
            cv::KeyPoint newKeyPoint;
            newKeyPoint.pt = cv::Point2f(i, j);
            newKeyPoint.size = 2 * apertureSize;
            newKeyPoint.response = response;

            // Perform non-maximum suppression (NMS) in local neighbourhood around the new keypoint
            bool bOverlap = false;
            // Loop over all existing keypoints
            for (auto it = keypoints.begin(); it != keypoints.end(); ++it) {
                double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                // Test if overlap exceeds the maximum percentage allowable
                if (kptOverlap > maxOverlap) {
                    bOverlap = true;
                    // If overlapping, test if new response is the local maximum
                    if (newKeyPoint.response > (*it).response) {
                        *it = newKeyPoint;  // Replace the old keypoint
                        break;  // Exit for loop
                    }
                }
            }
            // If above response threshold and not overlapping any other keypoint
            if (!bOverlap) {
                keypoints.push_back(newKeyPoint);  // Add to keypoints list
            }
        }
    }
}


// Detect keypoints in an image using more recent methods available in OpenCV
void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType,bool bVis=false)
{
    if (detectorType.compare("FAST") == 0) {
        // int threshold_FAST = 150;
        auto fast = cv::FastFeatureDetector::create();
        fast->detect(img, keypoints);
    }
    else if (detectorType.compare("BRISK") == 0) {
        // int threshold_BRISK = 200;
        auto brisk = cv::BRISK::create();
        brisk->detect(img, keypoints);
    }
    else if (detectorType.compare("ORB") == 0) {
        auto orb = cv::ORB::create();
        orb->detect(img, keypoints);
    }
    else if (detectorType.compare("AKAZE") == 0) {
        auto akaze = cv::AKAZE::create();
        akaze->detect(img, keypoints);
    }
    else if (detectorType.compare("SIFT") == 0) {
        auto sift = cv::xfeatures2d::SIFT::create();
        sift->detect(img, keypoints);
    }
    else {
        // Specified detectorType is unsupported
        throw std::invalid_argument(detectorType + " is not a valid detectorType");
    }

    // visualize features
    if (bVis){
        // Visualize the keypoints
        std::string windowName = detectorType + " keypoint detection results";
        cv::namedWindow(windowName);
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

#endif