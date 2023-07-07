#include "vslam/stereo.hpp"

StereoDisparity::StereoDisparity(ros::NodeHandle *nh)
{
    // Defining local paramtere so it will automatically delete once constructor is done
    // Not using afterwords in class
    int minDisparity, numDisparities, blockSize, p1, p2, disp12MaxDiff, preFilterCap, 
            uniquenessRatio, speckleWindowSize, speckleRange;
    int imgWidth, imgHeight;

    XmlRpc::XmlRpcValue transformParam;
    cv::Mat T_left_right = cv::Mat::eye(4,4,CV_64FC1);

    std::vector<double> leftCameraIntrinsic;
    std::vector<double> rightCameraIntrinsic;
    std::vector<double> leftCameraDistortion;
    std::vector<double> rightCameraDistortion;

    cv::Mat rectLeft, rectRight, projLeft, projRight, Q;


    // Getting all required parameters
    // camera parameters
    nh->getParam("left_camera/intrinsics", leftCameraIntrinsic);
    nh->getParam("right_camera/intrinsics", rightCameraIntrinsic);

    nh->getParam("left_camera/distortion", leftCameraDistortion);
    nh->getParam("right_camera/distortion", rightCameraDistortion);

    // camera transformation matrix
    nh->getParam("right_camera/T_left_right", transformParam);

    // stereo disparity parameters
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

    // preparing intrinsic and extrinsic matrix
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


    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            T_left_right.at<double>(i,j) = static_cast<double>(transformParam[i][j]);
        }
    }
    R = T_left_right.colRange(0,3).rowRange(0,3);
    t = T_left_right.col(3).rowRange(0,3);

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

void StereoDisparity::getRectifiedStereo(cv_bridge::CvImageConstPtr pLeftImg, cv_bridge::CvImageConstPtr pRightImg, 
                        cv::Mat& rectifiedLeft, cv::Mat& rectifiedRight)
{
    // perform stereo rectification
    cv::remap(pLeftImg->image, rectifiedLeft, rectifyMap[0][0], rectifyMap[0][1], cv::INTER_LINEAR);
    cv::remap(pRightImg->image, rectifiedRight, rectifyMap[1][0], rectifyMap[1][1], cv::INTER_LINEAR);
}

void StereoDisparity::getDisparityMap(cv::Mat& rectifiedLeft, cv::Mat& rectifiedRight, 
                                    cv::Mat& disparityMap)
{
    stereoSGBM_->compute(rectifiedLeft, rectifiedRight, disparityMap);
    cv::normalize(disparityMap, disparityMap, 0, 255, cv::NORM_MINMAX, CV_8UC1);
}