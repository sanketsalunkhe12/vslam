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

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/calib3d/calib3d_c.h"

class StereoDisparity
{
    public:
        StereoDisparity(ros::NodeHandle *nh)
        {
            ROS_INFO("Starting Stereo Depth and Disparity Node");

            nh->getParam("left_image_topic", leftImageTopic);
            nh->getParam("right_image_topic", rightImageTopic);
            
            nh->getParam("camera_name", cameraName);

            nh->getParam("left_camera/intrinsics", leftCameraIntrinsic);
            nh->getParam("right_camera/intrinsics", rightCameraIntrinsic);

            nh->getParam("left_camera/distortion", leftCameraDistortion);
            nh->getParam("right_camera/distortion", rightCameraDistortion);

            // camera transformation matrix
            nh->getParam("right_camera/T_left_right", transformParam);
            
            for(int i=0; i<4; i++)
            {
                for(int j=0; j<4; j++)
                {
                    T_left_right.at<double>(i,j) = static_cast<double>(transformParam[i][j]);
                }
            }
            R = T_left_right.colRange(0,3).rowRange(0,3);
            t = T_left_right.col(3).rowRange(0,3);

          
            // creating message filter time sync callback function
            leftImageSub.subscribe(*nh, leftImageTopic, 10);
            rightImageSub.subscribe(*nh, rightImageTopic, 10);

            imgSync.reset(new message_filters::TimeSynchronizer<sensor_msgs::Image, 
                            sensor_msgs::Image>(leftImageSub, rightImageSub, 10));
            imgSync->registerCallback(boost::bind(&StereoDisparity::stereoSyncCallback, 
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
            cv::namedWindow(disparityImgFrame);

            // creating stereo BM object
            nh->getParam("min_disparity", minDisparity);
            nh->getParam("num_disparity", numDisparities);
            nh->getParam("block_size", blockSize);
            nh->getParam("p1", p1);
            nh->getParam("p2", p2);
            nh->getParam("disp_12_max_diff", disp12MaxDiff);
            nh->getParam("pre_filter_cap", preFilterCap);
            nh->getParam("uniqueness_ratio", uniquenessRatio);
            nh->getParam("speckle_window_size", speckleWindowSize);
            nh->getParam("speckle_range", speckleRange);

            nh->getParam("left_camera/infra_width", imgWidth);
            nh->getParam("left_camera/infra_height", imgHeight);

            
            // getting stereo rectification params
            cv::stereoRectify(leftCameraCalibMat, leftDistortMat, rightCameraCalibMat, rightDistortMat, 
                            cv::Size(imgWidth, imgHeight), R, t, rectLeft, rectRight, projLeft, projRight,
                            Q, CV_CALIB_ZERO_DISPARITY, 0);

            // create a recitify mapping
            cv::initUndistortRectifyMap(leftCameraCalibMat, leftDistortMat, rectLeft, projLeft, 
                                        cv::Size(imgWidth, imgHeight), CV_16SC2, rectifyMap[0][0], rectifyMap[0][1]);
            
            cv::initUndistortRectifyMap(rightCameraCalibMat, rightDistortMat, rectRight, projRight,
                                        cv::Size(imgWidth, imgHeight), CV_16SC2, rectifyMap[1][0], rectifyMap[1][1]);
            
            // creating stereo object
            stereoSGBM_ = cv::StereoSGBM::create(minDisparity, 
                                        numDisparities, blockSize, p1, p2, disp12MaxDiff,
                                        preFilterCap, uniquenessRatio, speckleWindowSize,
                                        speckleRange, false);
        }

        ~StereoDisparity()
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

        cv::Mat leftCameraCalibMat, rightCameraCalibMat, leftDistortMat, rightDistortMat, R, t;
        
        // Undistorted images
        cv::Mat undistortLeftImg, undistortRightImg;
        
        // Image frame
        std::string cvLeftImgFrame = "Left Image";
        std::string cvRightImgFrame = "Right Image";

        std::string disparityImgFrame = "Disparity Image";

        // stereo disparity parameters
        cv::Ptr<cv::StereoSGBM> stereoSGBM_;
        int minDisparity, numDisparities, blockSize, p1, p2, disp12MaxDiff, preFilterCap, 
            uniquenessRatio, speckleWindowSize, speckleRange;
        int imgWidth, imgHeight;
        
        // rectification
        cv::Mat rectLeft, rectRight, projLeft, projRight, Q; //3*3 rectification transform, 3*4 projection matrix, 4*4 disparity-depth matrix 
        cv::Mat dispImg, dispImg_;  
        cv::Mat T_left_right = cv::Mat::eye(4,4,CV_64FC1);    
        XmlRpc::XmlRpcValue transformParam;
        cv::Mat rectifyMap[2][2];

        cv::Mat rectifiedLeft, rectifiedRight;
        
        
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

            // stereo rectification only perform a stereo transform rectification
            // we still need to perform intrinsic undistortion before feeding into this

            // perform stereo rectification
            cv::remap(pLeftImg->image, rectifiedLeft, rectifyMap[0][0], rectifyMap[0][1], cv::INTER_LINEAR);
            cv::remap(pRightImg->image, rectifiedRight, rectifyMap[1][0], rectifyMap[1][1], cv::INTER_LINEAR);

            // if want we can perform gaussian blur
                        
            // compute disparity
            stereoSGBM_->compute(rectifiedLeft, rectifiedRight, dispImg_);
            cv::normalize(dispImg_, dispImg, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            
            cv::imshow(cvLeftImgFrame, rectifiedLeft);
            cv::imshow(cvRightImgFrame, rectifiedRight);
                    
            cv::imshow(disparityImgFrame, dispImg);

            cv::waitKey(1);
        }
   

        void rectifyStereo()
        {

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

    StereoDisparity stereo_depth_disparity(&nh);
    ros::spin();

    return 0;
}