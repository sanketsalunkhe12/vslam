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

#include "vslam/ros_manager.hpp"
// #include "vslam/feature_detection.hpp"

class VisualSLAM
{
    public:
        VisualSLAM(ros::NodeHandle *nh)
        {
            ROS_INFO("Inside visual SLAM node");

            // ROSManager rosManager(nh);
            rosManager = std::make_shared<ROSManager>(nh);

        }
    private:
        std::shared_ptr<ROSManager> rosManager;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Visual_SLAM_Manager_node");
    ros::NodeHandle nh;

    VisualSLAM visual_SLAM(&nh);
    ros::spin();

    return 0;
}