#include "detector.h"
#include "matching.h"
#include "descriptor.h"
#include <deque>

struct DataFrame{

    cv::Mat cameraImg;//camera Image

    std::vector<cv::KeyPoint> keypoints; //2D keypoints within camera image
    cv::Mat descriptor; //keypoints descriptor
    std::vector<cv::DMatch> kptMatch; //keypoint match between two frames
};


int main(){
    // ORB,BRISK,FAST,AKAZE,SIFT
    std::string detectorType = "FAST";
    // std::string detectorType = "BRISK";
    // std::string detectorType = "ORB";
    // std::string detectorType = "AKAZE";
    // std::string detectorType = "SIFT";

    //BRISK,BRIEF,ORB,FREAK,SIFT
    // std::string descriptorType = "ORB";
    std::string descriptorType = "BRISK";
    // std::string descriptorType = "BRIEF";
    // std::string descriptorType = "FREAK";
    // std::string descriptorType = "SIFT";
    
    std::deque<DataFrame> dataBuffer;

    for(size_t i=0;i<10;++i){
        cv::Mat img = cv::imread("../image/000000000" + std::to_string(i) + ".png",0);
        assert(!img.empty());

        DataFrame frame;
        frame.cameraImg = img;   
        dataBuffer.push_back(frame);
        if(dataBuffer.size()>2)dataBuffer.pop_front();

        assert(dataBuffer.size() <=2);

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptor;
        detKeypointsModern(keypoints,img,detectorType);

        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle){
            std::vector<cv::KeyPoint> filteredKeypoints;
            for (auto kp : keypoints) {
                if (vehicleRect.contains(kp.pt)) filteredKeypoints.push_back(kp);
            }
            keypoints = filteredKeypoints;
        }

        bool bLimitKpts = false;
        if (bLimitKpts){
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            std::cout << " NOTE: Keypoints have been limited!" << std::endl;
        }

        (dataBuffer.end()-1)->keypoints = keypoints;//assign keypoints

        // calculate descriptors
        descKeypoints((dataBuffer.end()-1)->keypoints,(dataBuffer.end()-1)->cameraImg,descriptor,descriptorType);

        (dataBuffer.end()-1)->descriptor = descriptor;//assign descriptors

        if(dataBuffer.size()>1){
            std::vector<cv::DMatch> matches;
            
            std::string matcherType = "MAT_BF";
            // std::string matcherType = "MAT_FLANN";
            
            std::string descriptorCategory = {};
            if(descriptorType.compare("SIFT") == 0){
                descriptorCategory = "DES_HOG";
            }else{
                descriptorCategory = "DES_BINARY";
            }

            //K nearest neighbor search
            // std::string selectorType = "SEL_NN";
            std::string selectorType = "SEL_KNN";

            matchDescriptors((dataBuffer.end()-2)->keypoints,(dataBuffer.end()-1)->keypoints,
                    (dataBuffer.end()-2)->descriptor,(dataBuffer.end()-1)->descriptor,matches,descriptorCategory,matcherType,selectorType);

            (dataBuffer.end()-1)->kptMatch = matches;

            bool bVis = true;
            if (bVis){
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                std::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                std::string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                std::cout << "Press key to continue to next image" << std::endl;
                cv::waitKey(0); // wait for key to be pressed
                // cv::imwrite("res.png",matchImg);
            }
            bVis = false;
        }

    }

    return 0;
}