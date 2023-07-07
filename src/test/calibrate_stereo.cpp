#include "ros/ros.h"
#include <unistd.h>
#include <filesystem>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "sensor_msgs/Image.h"

#include "cv_bridge/cv_bridge.h"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"


// Idea: can be done using video instead of saving photo.

class StereoCalibration
{
    public:
        StereoCalibration(ros::NodeHandle *nh)
        {
            ROS_INFO("Starting stereo calibration");

            nh->getParam("calibrPath", calibrPath);
            nh->getParam("boardCols", boardCols);
            nh->getParam("boardRows", boardRows);
            nh->getParam("squareSize", squareSize);
            nh->getParam("numImg", numImg);
            nh->getParam("cam1Topic", cam1Topic);
            nh->getParam("cam2Topic", cam2Topic);

            left_img.subscribe(*nh, cam1Topic, 10);
            right_img.subscribe(*nh, cam2Topic, 10);

            imgSync.reset(new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>(left_img, right_img, 10));
            imgSync->registerCallback(boost::bind(&StereoCalibration::calibrationCallback, this, _1, _2));

            cv::namedWindow(cvLeftImgFrame);
            cv::namedWindow(cvRightImgFrame);
        }

        ~StereoCalibration()
        {
            cv::destroyAllWindows();
        }

        
    private:
        message_filters::Subscriber<sensor_msgs::Image> left_img;
        message_filters::Subscriber<sensor_msgs::Image> right_img;

        std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>> imgSync;

        std::string cvLeftImgFrame = "Left Image";
        std::string cvRightImgFrame = "Right Image";

        std::string calibrPath;
        std::string cam1Topic, cam2Topic;

        // Calibration chekerboard parameter
        int boardCols;
        int boardRows;
        float squareSize;
        int numImg;

        int imgCount = 0;

        // camera calibration
        cv::Mat leftCameraMatrix, leftDistCoefficient, rightCameraMatrix, rightDistCoefficient;
        std::vector<cv::Mat> leftRvecs, rightRvecs, leftTvecs, rightTvecs;
        
        bool distortion = true;

        cv::Mat undistortedLeftImage, undistortedRightImage;;

        void calibrationCallback(const sensor_msgs::ImageConstPtr& leftImg, const sensor_msgs::ImageConstPtr& rightImg)
        {
            cv_bridge::CvImageConstPtr pLeftImg, pRightImg;

            try
            {
                pLeftImg = cv_bridge::toCvCopy(leftImg);
                pRightImg = cv_bridge::toCvCopy(rightImg);
            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            if(distortion)
                distortImage(pLeftImg, pRightImg);
            else
                undistortImage(pLeftImg, pRightImg);
        }

        void undistortImage(cv_bridge::CvImageConstPtr& pLeftImg, cv_bridge::CvImageConstPtr& pRightImg)
        {
            // ROS_INFO("Undistorted images");

            cv::undistort(pLeftImg->image, undistortedLeftImage,leftCameraMatrix, leftDistCoefficient);
            cv::undistort(pRightImg->image, undistortedRightImage,rightCameraMatrix, rightDistCoefficient);

            cv::imshow(cvLeftImgFrame, undistortedLeftImage);
            cv::imshow(cvRightImgFrame, undistortedRightImage);

            cv::waitKey(1);
        }

        void distortImage(cv_bridge::CvImageConstPtr pLeftImg, cv_bridge::CvImageConstPtr pRightImg)
        {
            cv::imshow(cvLeftImgFrame, pLeftImg->image);
            cv::imshow(cvRightImgFrame, pRightImg->image);

            int key = cv::waitKey(1);
            if ((key == 's' || key == 'S') && imgCount <= numImg)
            {
                std::string leftFilename = calibrPath + "/left/left_image" +  std::to_string(imgCount) + ".jpg";
                std::string rightFilename = calibrPath + "/right/right_image" +  std::to_string(imgCount) + ".jpg";
                
                // While using same images comment out this
                cv::imwrite(leftFilename, pLeftImg->image);
                cv::imwrite(rightFilename, pRightImg->image);

                ROS_INFO("Image saved %d", imgCount);

                if(imgCount == numImg)
                {
                    intrinsicCalibration();
                }

                imgCount ++;
            }
        }

        void intrinsicCalibration()
        {
            ROS_INFO("Performing Intrinsic calibration");

            cv::Size boardSize(boardRows, boardCols);
            std::vector<std::vector<cv::Point2f>> leftImgPoints, rightImgPoints;
            std::vector<std::vector<cv::Point3f>> leftObjPoints, rightObjPoints;

            std::string leftFolderPath = calibrPath + "/left";
            std::string rightFolderPath = calibrPath + "/right";

            cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

            std::vector<cv::Point3f> objp;
            for(int i=0; i<boardCols; i++)
            {
                for(int j=0; j<boardRows; j++)
                {
                    objp.push_back(cv::Point3f(j*squareSize, i*squareSize, 0.0f));
                }
            }
            

            for(int imgCount=0; imgCount<=numImg; imgCount++)
            {
                std::string leftImgPath = leftFolderPath + "/left_image" + std::to_string(imgCount) + ".jpg";
                std::string rightImgPath = rightFolderPath + "/right_image" + std::to_string(imgCount) + ".jpg";

                cv::Mat leftImg = cv::imread(leftImgPath, cv::IMREAD_GRAYSCALE);
                cv::Mat rightImg = cv::imread(rightImgPath, cv::IMREAD_GRAYSCALE);

                std::vector<cv::Point2f> leftCorners, rightCorners;
                bool leftCornerFound = cv::findChessboardCorners(leftImg, boardSize, leftCorners,
                                                        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE 
                                                        + cv::CALIB_CB_FAST_CHECK);

                bool rightCornerFound = cv::findChessboardCorners(rightImg, boardSize, rightCorners,
                                                        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE 
                                                        + cv::CALIB_CB_FAST_CHECK);
                
                if(leftCornerFound)
                {                    
                    cv::cornerSubPix(leftImg, leftCorners, cv::Size(11,11), cv::Size(-1, -1),
                                    criteria);

                    // the process should stop either when the maximum number of iterations (30) is reached 
                    // or when the desired accuracy (0.1) is achieved.
                    
                    leftObjPoints.push_back(objp);
                    leftImgPoints.push_back(leftCorners);
                    cv::drawChessboardCorners(leftImg, boardSize, leftCorners, true);
                }

                if(rightCornerFound)
                {
                    cv::cornerSubPix(rightImg, rightCorners, cv::Size(11,11), cv::Size(-1, -1),
                                    criteria);
                    
                    rightObjPoints.push_back(objp);
                    rightImgPoints.push_back(rightCorners);
                    cv::drawChessboardCorners(rightImg, boardSize, rightCorners, true);
                }

                cv::imshow(cvLeftImgFrame, leftImg);
                cv::imshow(cvRightImgFrame, rightImg);

                cv::waitKey(100);
            }

            // Getting image size
            std::string leftImgPath = leftFolderPath + "/left_image0.jpg";
            std::string rightImgPath = rightFolderPath + "/right_image0.jpg";

            cv::Mat leftImg = cv::imread(leftImgPath, cv::IMREAD_GRAYSCALE);
            cv::Mat rightImg = cv::imread(rightImgPath, cv::IMREAD_GRAYSCALE);
            
            // checking
            ROS_INFO("Image size %d %d",leftImg.rows, leftImg.cols);

            double leftRMS = cv::calibrateCamera(leftObjPoints, leftImgPoints, 
                                leftImg.size(),
                                leftCameraMatrix, leftDistCoefficient, leftRvecs, leftTvecs);
            double rightRMS = cv::calibrateCamera(rightObjPoints, rightImgPoints, 
                                rightImg.size(),
                                rightCameraMatrix, rightDistCoefficient, rightRvecs, rightTvecs);

            distortion = false;
            
            std::cout << "Left Camera Matrix:\n" << leftCameraMatrix << std::endl;
            std::cout << "Right Camera Matrix:\n" << rightCameraMatrix << std::endl;
            std::cout << "Left Distortion Coefficients:\n" << leftDistCoefficient << std::endl;
            std::cout << "Right Distortion Coefficients:\n" << rightDistCoefficient << std::endl;
        }

        void stereoCalibration()
        {
            ROS_INFO("Performing stereo calibration");
        }
        
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Stereo_Camera_Calibration_Node");
    ros::NodeHandle nh;

    StereoCalibration stereo_calibration(&nh);
    ros::spin();

    return 0;
}