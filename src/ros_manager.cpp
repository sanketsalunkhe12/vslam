#include "ros/ros.h"
#include "ros/console.h"

#include <thread>
#include <future>

#include "vslam/ros_manager.hpp"
#include "vslam/feature_detection.hpp"
#include "vslam/stereo.hpp"
#include "vslam/camera_calib.hpp"
#include "vslam/feature_tracking.hpp"

ROSManager::ROSManager(ros::NodeHandle *nh)
{
    ROS_INFO("Initialize ROS Manager");

    std::string leftImageTopic;
    std::string rightImageTopic;
    std::string imuTopic;

    // Getting all ROS parameters
    // need to write failsafe error waiting message
    nh->getParam("left_image_topic", leftImageTopic);
    nh->getParam("right_image_topic", rightImageTopic);
    nh->getParam("imu_topic", imuTopic);
    
    nh->getParam("camera_name", cameraName);

    // creating message filter time sync callback function
    leftImageSub.subscribe(*nh, leftImageTopic, 10);
    rightImageSub.subscribe(*nh, rightImageTopic, 10);

    imgSync.reset(new message_filters::TimeSynchronizer<sensor_msgs::Image, 
                    sensor_msgs::Image>(leftImageSub, rightImageSub, 1));
    imgSync->registerCallback(boost::bind(&ROSManager::stereoSyncCallback, 
                                    this, _1, _2));

    // Creating IMU callback function
    imuSub = nh->subscribe(imuTopic, 100, &ROSManager::imuCallback, this);

    featureDetection = new FeatureDetection(nh);

    stereoDisparity = new StereoDisparity(nh);

    cameraCalibration = new CameraCalibration(nh);

    featureTracking = new FeatureTracking(nh);

}


void ROSManager::stereoSyncCallback(const sensor_msgs::ImageConstPtr& leftImage, 
                                const sensor_msgs::ImageConstPtr& rightImage)
{
    // ROS_ERROR("Getting stereo images");
    try
    {
        pLeftImg = cv_bridge::toCvCopy(leftImage);
        pRightImg = cv_bridge::toCvCopy(rightImage);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // ros::Time startTime = ros::Time::now();
    
    auto calibThrL = std::async(std::launch::async, &CameraCalibration::getCalibratedLeftImg,
                                cameraCalibration, pLeftImg, std::ref(undistortLeftImg));

    auto calibThrR = std::async(std::launch::async, &CameraCalibration::getCalibratedRightImg,
                                cameraCalibration, pRightImg, std::ref(undistortRightImg));

    calibThrL.wait();
    calibThrR.wait();
    
    // cameraCalibration->getCalibratedLeftImg(pLeftImg, undistortLeftImg);
    // cameraCalibration->getCalibratedLeftImg(pRightImg, undistortRightImg);

    stereoDisparity->getRectifiedStereo(undistortLeftImg, undistortRightImg, 
                                        rectifiedLeft, rectifiedRight);

    
    auto dispMapThr = std::async(std::launch::async, &StereoDisparity::getDisparityMap, stereoDisparity, 
                            std::ref(rectifiedLeft), std::ref(rectifiedRight), std::ref(disparityMap));

    auto featDectThr = std::async(std::launch::async, &FeatureDetection::ORBFeatureDetector, featureDetection, 
                            std::ref(rectifiedLeft), std::ref(distKeypoint));

    dispMapThr.wait();
    featDectThr.wait();
    
    // stereoDisparity->getDisparityMap(rectifiedLeft, rectifiedRight, disparityMap);
    // featureDetection->ORBFeatureDetector(rectifiedLeft, distKeypoint);

    featureTracking->trackFeatures(rectifiedLeft, distKeypoint);

    // ros::Time endTime = ros::Time::now();
    // ros::Duration duration = endTime - startTime;
    // ROS_ERROR("The duration is %f", duration.toSec());
    
}


void ROSManager::imuCallback(const sensor_msgs::ImuConstPtr& imuMessage)
{
    // ROS_INFO("getting IMU messages");
}


