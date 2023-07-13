#include <iostream>
#include "ros/ros.h"
#include "vslam/feature_tracking.hpp"

FeatureTracking::FeatureTracking(ros::NodeHandle *nh)
{
    ROS_INFO("Initializing feature tracking");

    nh->getParam("klt_window_width", winWidth);
    nh->getParam("klt_window_hight", winHight);
    nh->getParam("max_pyramid_level", maxPyrLevel);
    nh->getParam("min_tracked_features", minFeatTrack);

    nh->getParam("distance_threshold", distanceThreshold);
    nh->getParam("confidence",confidence);
}

void FeatureTracking::trackFeatures(cv::Mat& currentImg, 
                                std::vector<cv::KeyPoint>& distKeypoint)
{
    if(trackingInitialize)
    {
        // ROS_INFO("Tracking initialize");
        cv::Size winSize(winWidth, winHight);

        std::vector<cv::Point2f> currentFrameKeyPts;
        std::vector<cv::Point2f> prevFrameTrackedPts, currentFrameTrackedPts;
        std::vector<cv::Point2f> prevFrameInlier, currentFrameInlier;
        std::vector<uchar> maskRANSAC;
        
        if(prevFrameKeyPts.size() > minFeatTrack)
        {
            // calculating optical flow
            cv::calcOpticalFlowPyrLK(prevImg, currentImg, prevFrameKeyPts, currentFrameKeyPts, 
                            status, error, winSize, maxPyrLevel, criteria);

            for(uint i = 0; i<prevFrameKeyPts.size(); i++)
            {
                if(status[i] == 1)
                {
                    prevFrameTrackedPts.push_back(prevFrameKeyPts[i]);
                    currentFrameTrackedPts.push_back(currentFrameKeyPts[i]);
                }
            }

            // remove outliers using RANSAC
            cv::findFundamentalMat(prevFrameTrackedPts, currentFrameTrackedPts, cv::FM_RANSAC, 
                                    distanceThreshold, confidence, maskRANSAC);

            int count = 0;
            for(uint i=0; i<prevFrameTrackedPts.size(); i++)
            {
                if(maskRANSAC[i] != 1)
                    continue;
                
                prevFrameInlier.push_back(prevFrameTrackedPts[i]);
                currentFrameInlier.push_back(currentFrameTrackedPts[i]);

                cv::line(prevImg, prevFrameInlier[count], currentFrameInlier[count], 
                        cv::Scalar(0, 255, 0), 2);
                cv::circle(prevImg, currentFrameInlier[count], 3, cv::Scalar(0, 0, 255), -1);
                count++;
            }
        }
        else
        {
            ROS_ERROR("Not enough features");
        }
        
        // selecting good points     
        cv::Mat orbOutputImg;
        cv::drawKeypoints(prevImg, distKeypoint, prevImg);

        cv::imshow("Feature tracking", prevImg);
        cv::waitKey(1);

        prevImg = currentImg.clone();
        prevFrameFeatures = distKeypoint;
        cv::KeyPoint::convert(prevFrameFeatures, prevFrameKeyPts, prevKeyPtsIndex);
    }
    else
    {
        // ROS_INFO("Initiate tracking process");
        prevImg = currentImg.clone();
        prevFrameFeatures = distKeypoint;

        cv::KeyPoint::convert(prevFrameFeatures, prevFrameKeyPts, prevKeyPtsIndex);
        trackingInitialize = true;
    }
}