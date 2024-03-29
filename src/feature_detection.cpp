#include "ros/ros.h"
#include "ros/console.h"

#include "vslam/feature_detection.hpp"
// #include "vslam/ros_manager.hpp"
#include "vslam/ssc_anms.hpp"


FeatureDetection::FeatureDetection(ros::NodeHandle *nh)//, ROSManager* rosManager)
{
    ROS_INFO("Initialize feature detection");

    nh->getParam("/numRetPoints", numRetPoints);
    nh->getParam("/tolerance", tolerance);
    nh->getParam("/NumFeatures", NumFeatures);
    // nh->getParam("/numBestMatch", numBestMatch);

    KeypointDescriptor orbKeyDisImg1, orbKeyDisImg2;

}


// FeatureDetection::KeypointDescriptor 
void FeatureDetection::ORBFeatureDetector(cv::Mat& undistortLeftImg, 
                                    std::vector<cv::KeyPoint>& distKeypoint)
{
    // ORB is both Feature detector and descriptor
    cv::Ptr<cv::ORB> orbDetector = cv::ORB::create(NumFeatures);
    std::vector<cv::KeyPoint> orbKeypoint; 

    // computing keypoint
    orbDetector->detect(undistortLeftImg, orbKeypoint); 

    // homogenous spatial-keypoints-distribution
    std::sort(orbKeypoint.begin(), orbKeypoint.end(), CompareKeypointResponse);

    // from ssc_anms.hpp
    distKeypoint = DistributedKeypoint(orbKeypoint, numRetPoints, 
                    tolerance, undistortLeftImg.cols, undistortLeftImg.rows);

    // std::string sizeStr = std::to_string(distKeypoint.size());
    // ROS_INFO("Size of the vector: %s", sizeStr.c_str());
    
    // //compute descriptor
    // cv::Mat orbDescriptor;
    // orbDetector->compute(undistortLeftImg, distKeypoint, orbDescriptor); 
    
    // cv::Mat orbOutputImg;
    // cv::drawKeypoints(undistortLeftImg, distKeypoint, orbOutputImg);

    // cv::imshow("ORB detector", orbOutputImg);
    // cv::waitKey(1);
}