#ifndef FEATURE_TRACKING_HPP
#define FEATURE_TRACKING_HPP

#include "ros/ros.h"
#include "ros/console.h"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"

#include "opencv2/features2d.hpp"

#include "vslam/keyframe.hpp"

class FeatureTracking
{
    public:
        FeatureTracking(ros::NodeHandle *nh);

        void trackFeatures(cv::Mat& currentImg, std::vector<cv::KeyPoint>& distKeypoint);

        void stereoInitialize();
    
    private:
        int winWidth, winHight, maxPyrLevel, minFeatTrack;
        double distanceThreshold, confidence;
        
        bool trackingInitialize = false;
        std::vector<uchar> status;
        std::vector<float> error;

        cv::Mat prevImg;
        std::vector<int> prevKeyPtsIndex, currentKeyPtsIndex;
        std::vector<cv::Point2f> prevFrameKeyPts, currentFrameKeyPts;
        std::vector<cv::KeyPoint> prevFrameFeatures, currentFrameFeatures;

        cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 
                                    30, 0.01);

        
        // current and last frame initialization 
        // of type Frame
        
};

#endif