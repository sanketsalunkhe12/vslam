#include "ros/ros.h"

#include "cv_bridge/cv_bridge.h"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"

class CameraCalibration
{
    public:
        CameraCalibration(ros::NodeHandle *nh);

        void getCalibratedLeftImg(cv_bridge::CvImageConstPtr pLeftImg, 
                                cv::Mat& undistortLeftImg);

        void getCalibratedRightImg(cv_bridge::CvImageConstPtr pRightImg, 
                                cv::Mat& undistortRightImg);

    private:
        cv::Mat leftCameraCalibMat, rightCameraCalibMat, leftDistortMat, 
                rightDistortMat;
};
