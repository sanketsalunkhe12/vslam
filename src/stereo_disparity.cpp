#include "ros/ros.h"

#include "eigen3/Eigen/Dense"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "sensor_msgs/Image.h"

#include "cv_bridge/cv_bridge.h"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"

class StereoDepthDisparity
{
    public:
        StereoDepthDisparity(ros::NodeHandle *nh)
        {
            ROS_INFO("Starting Stereo Depth and Disparity Node");

            nh->getParam("left_image_topic", leftImageTopic);
            nh->getParam("right_image_topic", rightImageTopic);
            
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
            imgSync->registerCallback(boost::bind(&StereoDepthDisparity::stereoSyncCallback, 
                                            this, _1, _2));

            // Camera calibration matrix
            leftCameraCalibMat = (cv::Mat_<double>(3,3) << leftCameraIntrinsic[0], 0.0, leftCameraIntrinsic[2],
                                                    0.0, leftCameraIntrinsic[1], leftCameraIntrinsic[3],
                                                    0.0, 0.0, 1.0);

            rightCameraCalibMat = (cv::Mat_<double>(3,3) << rightCameraIntrinsic[0], 0.0, rightCameraIntrinsic[2],
                                                    0.0, rightCameraIntrinsic[1], rightCameraIntrinsic[3],
                                                    0.0, 0.0, 1.0);

            leftDistortMat = (cv::Mat_<double>(1,5) << leftCameraDistortion[0], leftCameraDistortion[1], 
                                leftCameraDistortion[2], leftCameraDistortion[3], leftCameraDistortion[4]);

            rightDistortMat = (cv::Mat_<double>(1,5) << rightCameraDistortion[0], rightCameraDistortion[1], 
                                rightCameraDistortion[2], rightCameraDistortion[3], rightCameraDistortion[4]);

            // imshow windows
            cv::namedWindow(cvLeftImgFrame);
            cv::namedWindow(cvRightImgFrame);

        }

        ~StereoDepthDisparity()
        {
            cv::destroyAllWindows();
        }

    private:

        std::string leftImageTopic;
        std::string rightImageTopic;
        
        std::string cameraName;
        std::vector<double> leftCameraIntrinsic;
        std::vector<double> rightCameraIntrinsic;
        std::vector<double> leftCameraDistortion;
        std::vector<double> rightCameraDistortion;

        message_filters::Subscriber<sensor_msgs::Image> leftImageSub;
        message_filters::Subscriber<sensor_msgs::Image> rightImageSub;

        std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, 
                                                sensor_msgs::Image>> imgSync;

        cv::Mat leftCameraCalibMat, rightCameraCalibMat, leftDistortMat, rightDistortMat;
        
        // Undistorted images
        cv::Mat undistortLeftImg, undistortRightImg;
        
        // Image frame
        std::string cvLeftImgFrame = "Left Image";
        std::string cvRightImgFrame = "Right Image";
              
        
        void stereoSyncCallback(const sensor_msgs::ImageConstPtr& leftImage, 
                                const sensor_msgs::ImageConstPtr& rightImage)
        {
            // ROS_INFO("Getting stereo images");

            cv_bridge::CvImageConstPtr pLeftImg, pRightImg;
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

            undistortImage(pLeftImg, pRightImg);
            
            cv::imshow(cvLeftImgFrame, undistortLeftImg);
            cv::imshow(cvRightImgFrame, undistortRightImg);

            cv::waitKey(1);
        }
   

        void undistortImage(cv_bridge::CvImageConstPtr pLeftImg, cv_bridge::CvImageConstPtr pRightImg)
        {
            cv::undistort(pLeftImg->image, undistortLeftImg, leftCameraCalibMat, leftDistortMat);
            cv::undistort(pRightImg->image, undistortRightImg, rightCameraCalibMat, rightDistortMat);
        }



};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Stereo_depth_and_disparity_node");
    ros::NodeHandle nh;

    StereoDepthDisparity stereo_depth_disparity(&nh);
    ros::spin();

    return 0;
}