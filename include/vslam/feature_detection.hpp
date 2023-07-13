#ifndef FEATURE_DETECTION_HPP
#define FEATURE_DETECTION_HPP

#include "ros/ros.h"
#include "ros/console.h"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"

#include "opencv2/features2d.hpp"

#include "cv_bridge/cv_bridge.h"

#include <iostream>

class ROSManager;

class FeatureDetection
{
    public:
        FeatureDetection(ros::NodeHandle *nh);

        struct KeypointDescriptor
        {
            std::vector<cv::KeyPoint> keypoint;
            cv::Mat descriptor;
        };
        
        // KeypointDescriptor 
        void ORBFeatureDetector(cv::Mat& undistortLeftImg, std::vector<cv::KeyPoint>& distKeypoint);

    private:
        // SSC ANMS Parameters
        int numRetPoints;
        float tolerance;

        // ORB Parameters
        int NumFeatures;

        static bool CompareKeypointResponse(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2)
        {
            return kp1.response > kp2.response;
        }

};

#endif // FEATURE_DETECTION_HPP