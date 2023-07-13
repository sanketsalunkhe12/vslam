#ifndef STEREO_HPP
#define STEREO_HPP

#include "ros/ros.h"

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
        StereoDisparity(ros::NodeHandle *nh);

        void getRectifiedStereo(cv::Mat& undistortLeftImg, cv::Mat& undistortRightImg,
                        cv::Mat& rectifiedLeft, cv::Mat& rectifiedRight);

        void getDisparityMap(cv::Mat& rectifiedLeft, cv::Mat& rectifiedRight, 
                            cv::Mat& disparityMap);

    private:
        cv::Mat leftCameraCalibMat, rightCameraCalibMat, leftDistortMat, 
                rightDistortMat, R, t;

        cv::Mat rectifyMap[2][2];

        cv::Ptr<cv::StereoSGBM> stereoSGBM_;

};

#endif