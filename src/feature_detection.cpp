#include "ros/ros.h"
#include "ros/console.h"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"

#include "opencv2/features2d.hpp"

#include <iostream>

#include "vslam/ssc_anms.hpp"

class FeatureDetection
{
    public:
        FeatureDetection(ros::NodeHandle nh)
        {
            ROS_INFO("Feature detection");

            // Loading required param
            nh.getParam("/numRetPoints", numRetPoints);
            nh.getParam("/tolerance", tolerance);
            nh.getParam("/NumFeatures", NumFeatures);
            nh.getParam("/numBestMatch", numBestMatch);

            nh.getParam("/imgPath", imgPath);
            nh.getParam("/imgPath2", imgPath2);

            cv::Mat grayImg = cv::imread(imgPath, cv::IMREAD_REDUCED_GRAYSCALE_2);
            cv::Mat grayImg2 = cv::imread(imgPath2, cv::IMREAD_REDUCED_GRAYSCALE_2);

            if(grayImg.empty())
            {
                ROS_ERROR("Could not read the image");
            }

            // harrisCornerDetector(grayImg);
            // fastFeatureDetector(grayImg);

            KeypointDescriptor orbKeyDisImg1, orbKeyDisImg2;

            orbKeyDisImg1 = ORBFeatureDetector(grayImg);
            orbKeyDisImg2 = ORBFeatureDetector(grayImg2);

            std::vector<cv::DMatch> matches = FlannMatcher(orbKeyDisImg1.descriptor, 
                                                    orbKeyDisImg2.descriptor);

            cv::Mat matchImage;
            cv::drawMatches(grayImg, orbKeyDisImg1.keypoint, grayImg2, orbKeyDisImg2.keypoint, 
                                matches, matchImage, cv::Scalar::all(-1), cv::Scalar::all(-1)),
                                std::vector< char >(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS;
            cv::imshow("match image", matchImage);          
            
            // cv::Mat outputImg, outputImg2;

            // cv::drawKeypoints(grayImg, orbKeyDisImg1.keypoint, outputImg);
            // cv::imshow("Output image", outputImg);

            // cv::drawKeypoints(grayImg2, orbKeyDisImg2.keypoint, outputImg2);
            // cv::imshow("Output image2", outputImg2);

            cv::waitKey(0);
        }
    private:

        // SSC ANMS Parameters
        int numRetPoints;
        float tolerance;

        std::string imgPath;
        std::string imgPath2;

        // ORB Parameters
        int NumFeatures;

        // FLANN Paramters
        int numBestMatch;

        struct KeypointDescriptor
        {
            std::vector<cv::KeyPoint> keypoint;
            cv::Mat descriptor;
        };
        
        static bool CompareKeypointResponse(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2)
        {
            return kp1.response > kp2.response;
        }

        static bool CompareMatchesDistance(const cv::DMatch& mat1, const cv::DMatch& mat2)
        {
            return mat1.distance < mat2.distance;
        }
        
        KeypointDescriptor ORBFeatureDetector(cv::Mat grayImg)
        {
            // ORB is both Feature detector and descriptor
            cv::Ptr<cv::ORB> orbDetector = cv::ORB::create(NumFeatures);
            std::vector<cv::KeyPoint> orbKeypoint;         

            orbDetector->detect(grayImg, orbKeypoint); // computing keypoint

            // homogenous spatial-keypoints-distribution
            std::sort(orbKeypoint.begin(), orbKeypoint.end(), CompareKeypointResponse);

            // from ssc_anms.hpp
            std::vector<cv::KeyPoint> distKeypoint = 
                    DistributedKeypoint(orbKeypoint, numRetPoints, tolerance, grayImg.cols, grayImg.rows);

            cv::Mat orbDescriptor;
            orbDetector->compute(grayImg, distKeypoint, orbDescriptor); //compute descriptor

            // cv::Mat orbDistOutputImg;
            // cv::drawKeypoints(grayImg, distKeypoint, orbDistOutputImg);

            // cv::Mat orbOutputImg;
            // cv::drawKeypoints(grayImg, orbKeypoint, orbOutputImg);

            // ROS_INFO("size of keypoint %d",distKeypoint.size());

            // cv::imshow("ORB Dist detector", orbDistOutputImg);
            // cv::imshow("ORB detector", orbOutputImg);

            // cv::waitKey(0);
            
            KeypointDescriptor orbKeyDes;
            orbKeyDes.keypoint = distKeypoint;
            orbKeyDes.descriptor = orbDescriptor;

            return orbKeyDes;
        }
        
        std::vector<cv::DMatch> FlannMatcher(cv::Mat orbDescriptor1, cv::Mat orbDescriptor2)
        {
            cv::FlannBasedMatcher flannMatcher;
            std::vector<cv::DMatch> matches;
            std::vector<cv::DMatch> topMatches;

            orbDescriptor1.convertTo(orbDescriptor1, CV_32F);
            orbDescriptor2.convertTo(orbDescriptor2, CV_32F);

            flannMatcher.match(orbDescriptor1, orbDescriptor2, matches);

            // Sort best min distance in ascending order
            std::sort(matches.begin(), matches.end(), CompareMatchesDistance);
            
            for (int i=0; i<numBestMatch; i++)
            {
                ROS_INFO("best matches %f", matches[i].distance);
                topMatches.push_back(matches[i]);
            }
            return topMatches;
        }       

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Feature Detection Node");
    ros::NodeHandle nh;
    
    FeatureDetection feature_detect(nh);
    return 0;
}