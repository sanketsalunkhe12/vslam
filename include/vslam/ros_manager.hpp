#ifndef ROS_MANAGER_HPP
#define ROS_MANAGER_HPP

#include "ros/ros.h"
#include "ros/console.h"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"

#include "cv_bridge/cv_bridge.h"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"

class FeatureDetection;
class StereoDisparity;
class CameraCalibration;
class FeatureTracking;

class ROSManager
{
    public:
        ROSManager(ros::NodeHandle *nh);

        ~ROSManager()
        {
            cv::destroyAllWindows();
        }

    private:
        FeatureDetection* featureDetection;
        StereoDisparity* stereoDisparity;
        CameraCalibration* cameraCalibration;
        FeatureTracking* featureTracking;

        std::string cameraName;

        cv::Mat undistortLeftImg, undistortRightImg;
        cv::Mat rectifiedLeft, rectifiedRight;
        cv::Mat disparityMap;
        std::vector<cv::KeyPoint> distKeypoint;

        // Sync Camera images
        message_filters::Subscriber<sensor_msgs::Image> leftImageSub;
        message_filters::Subscriber<sensor_msgs::Image> rightImageSub;

        std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, 
                                                sensor_msgs::Image>> imgSync;

        cv_bridge::CvImageConstPtr pLeftImg, pRightImg;

        // Image frame
        std::string cvLeftImgFrame = "Left Image";
        std::string cvRightImgFrame = "Right Image";

        // IMU 
        ros::Subscriber imuSub;

        void stereoSyncCallback(const sensor_msgs::ImageConstPtr& leftImage, 
                                const sensor_msgs::ImageConstPtr& rightImage);


        void imuCallback(const sensor_msgs::ImuConstPtr& imuMessage);


};

#endif // ROS_MANAGER_HPP