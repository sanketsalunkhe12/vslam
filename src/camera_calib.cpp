#include "vslam/camera_calib.hpp"

CameraCalibration::CameraCalibration(ros::NodeHandle *nh)
{
    ROS_INFO("Initialize camera calibration");

    std::vector<double> leftCameraIntrinsic;
    std::vector<double> rightCameraIntrinsic;
    std::vector<double> leftCameraDistortion;
    std::vector<double> rightCameraDistortion;

    nh->getParam("left_camera/intrinsics", leftCameraIntrinsic);
    nh->getParam("right_camera/intrinsics", rightCameraIntrinsic);

    nh->getParam("left_camera/distortion", leftCameraDistortion);
    nh->getParam("right_camera/distortion", rightCameraDistortion);

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

}

void CameraCalibration::getCalibratedLeftImg(cv_bridge::CvImageConstPtr pLeftImg, cv::Mat& undistortLeftImg)
{
    cv::undistort(pLeftImg->image, undistortLeftImg, leftCameraCalibMat, leftDistortMat);
}


void CameraCalibration::getCalibratedRightImg(cv_bridge::CvImageConstPtr pRightImg, cv::Mat& undistortRightImg)
{
    cv::undistort(pRightImg->image, undistortRightImg, rightCameraCalibMat, rightDistortMat);
}