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

class ROSManager
{
    public:
        ROSManager(ros::NodeHandle *nh);

        ~ROSManager()
        {
            // delete featureDetection;
            cv::destroyAllWindows();
        }

        cv::Mat getLeftCameraCalib();
        cv::Mat getRightCameraCalib();
        cv::Mat getLeftCameraDistort();
        cv::Mat getRightCameraDistord();

    private:
        FeatureDetection* featureDetection;
        StereoDisparity* stereoDisparity;

        std::string leftImageTopic;
        std::string rightImageTopic;
        std::string imuTopic;
        
        std::string cameraName;
        std::vector<double> leftCameraIntrinsic;
        std::vector<double> rightCameraIntrinsic;
        std::vector<double> leftCameraDistortion;
        std::vector<double> rightCameraDistortion;

        // Sync Camera images
        message_filters::Subscriber<sensor_msgs::Image> leftImageSub;
        message_filters::Subscriber<sensor_msgs::Image> rightImageSub;

        std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, 
                                                sensor_msgs::Image>> imgSync;

        cv_bridge::CvImageConstPtr pLeftImg, pRightImg;

        // Undistortion
        cv::Mat leftCameraCalibMat, rightCameraCalibMat, 
                leftDistortMat, rightDistortMat;
                
        // Image frame
        std::string cvLeftImgFrame = "Left Image";
        std::string cvRightImgFrame = "Right Image";

        // IMU 
        ros::Subscriber imuSub;

        void stereoSyncCallback(const sensor_msgs::ImageConstPtr& leftImage, 
                                const sensor_msgs::ImageConstPtr& rightImage);


        void imuCallback(const sensor_msgs::ImuConstPtr& imuMessage);


        void undistortImage(cv_bridge::CvImageConstPtr pLeftImg, cv_bridge::CvImageConstPtr pRightImg);

};

#endif // ROS_MANAGER_HPP