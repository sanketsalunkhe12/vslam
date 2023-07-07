#include "ros/ros.h"
#include "ros/console.h"

#include "vslam/ros_manager.hpp"
#include "vslam/feature_detection.hpp"
#include "vslam/stereo.hpp"

ROSManager::ROSManager(ros::NodeHandle *nh)
{
    ROS_INFO("Initialize ROS Manager");

    // Getting all ROS parameters
    // need to write failsafe error waiting message
    nh->getParam("left_image_topic", leftImageTopic);
    nh->getParam("right_image_topic", rightImageTopic);
    nh->getParam("imu_topic", imuTopic);
    
    nh->getParam("camera_name", cameraName);
    nh->getParam("left_intrinsic_param", leftCameraIntrinsic);
    nh->getParam("right_intrinsic_param", rightCameraIntrinsic);
    nh->getParam("left_distortion_param", leftCameraDistortion);
    nh->getParam("right_distortion_param", rightCameraDistortion);

    // creating message filter time sync callback function
    leftImageSub.subscribe(*nh, leftImageTopic, 10);
    rightImageSub.subscribe(*nh, rightImageTopic, 10);

    imgSync.reset(new message_filters::TimeSynchronizer<sensor_msgs::Image, 
                    sensor_msgs::Image>(leftImageSub, rightImageSub, 10));
    imgSync->registerCallback(boost::bind(&ROSManager::stereoSyncCallback, 
                                    this, _1, _2));

    // Creating IMU callback function
    imuSub = nh->subscribe(imuTopic, 100, &ROSManager::imuCallback, this);

    featureDetection = new FeatureDetection(nh, this);

    stereoDisparity = new StereoDisparity(nh);
    
        
    // imshow windows
    // cv::namedWindow(cvLeftImgFrame);
    // cv::namedWindow(cvRightImgFrame);
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

    // FeatureDetection::KeypointDescriptor orbKeyDes = 
    
    featureDetection->ORBFeatureDetector(pLeftImg);
    
    
    
    // undistortImage(pLeftImg, pRightImg);

    // cv::imshow(cvLeftImgFrame, pLeftImg->image);
    // cv::imshow(cvRightImgFrame, pRightImg->image);

    // cv::waitKey(1);
}


void ROSManager::imuCallback(const sensor_msgs::ImuConstPtr& imuMessage)
{
    // ROS_INFO("getting IMU messages");
}


void ROSManager::undistortImage(cv_bridge::CvImageConstPtr pLeftImg, cv_bridge::CvImageConstPtr pRightImg)
{
    // cv::undistort(pLeftImg->image, undistortLeftImg, leftCameraCalibMat, leftDistortMat);
    // cv::undistort(pRightImg->image, undistortRightImg, rightCameraCalibMat, rightDistortMat);
}


cv::Mat ROSManager::getLeftCameraCalib()
{
    leftCameraCalibMat = (cv::Mat_<double>(3,3) << leftCameraIntrinsic[0], 0.0, leftCameraIntrinsic[2],
                                            0.0, leftCameraIntrinsic[1], leftCameraIntrinsic[3],
                                            0.0, 0.0, 1.0);
    return leftCameraCalibMat;
}

cv::Mat ROSManager::getRightCameraCalib()
{
    rightCameraCalibMat = (cv::Mat_<double>(3,3) << rightCameraIntrinsic[0], 0.0, rightCameraIntrinsic[2],
                                            0.0, rightCameraIntrinsic[1], rightCameraIntrinsic[3],
                                            0.0, 0.0, 1.0);
    return rightCameraCalibMat;
}

cv::Mat ROSManager::getLeftCameraDistort()
{
    leftDistortMat = (cv::Mat_<double>(1,5) << leftCameraDistortion[0], leftCameraDistortion[1], 
                        leftCameraDistortion[2], leftCameraDistortion[3], leftCameraDistortion[4]);
    return leftDistortMat;
}

cv::Mat ROSManager::getRightCameraDistord()
{
    rightDistortMat = (cv::Mat_<double>(1,5) << rightCameraDistortion[0], rightCameraDistortion[1], 
                        rightCameraDistortion[2], rightCameraDistortion[3], rightCameraDistortion[4]);
    return rightDistortMat;
}
