#include "ros/ros.h"
#include "ros/console.h"

#include "vslam/ros_manager.hpp"
#include "vslam/feature_detection.hpp"
#include "vslam/stereo.hpp"
#include "vslam/camera_calib.hpp"

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
                    sensor_msgs::Image>(leftImageSub, rightImageSub, 10));
    imgSync->registerCallback(boost::bind(&ROSManager::stereoSyncCallback, 
                                    this, _1, _2));

    // Creating IMU callback function
    imuSub = nh->subscribe(imuTopic, 100, &ROSManager::imuCallback, this);

    featureDetection = new FeatureDetection(nh);

    stereoDisparity = new StereoDisparity(nh);

    cameraCalibration = new CameraCalibration(nh);

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
    
    featureDetection->ORBFeatureDetector(pLeftImg);
    
}


void ROSManager::imuCallback(const sensor_msgs::ImuConstPtr& imuMessage)
{
    // ROS_INFO("getting IMU messages");
}


