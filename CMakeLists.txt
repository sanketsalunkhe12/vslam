cmake_minimum_required(VERSION 3.0.2)
project(vslam)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  message_filters
  sensor_msgs
  cv_bridge
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES vslam  
    CATKIN_DEPENDS roscpp image_transport message_filters sensor_msgs cv_bridge
    # DEPENDS system_lib
)

include_directories(
    include
    data
    config
    ${catkin_INCLUDE_DIRS}
)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

list(APPEND third_party_lib
            ${OpenCV_LIBS}
            ${EIGEN3_LIBRARIES}
            ${catkin_LIBRARIES})

list(APPEND LIBRARY_SOURCES 
            src/ros_manager.cpp
            src/feature_detection.cpp
            src/stereo.cpp
            src/camera_calib.cpp)

add_library(vslam_libraries SHARED ${LIBRARY_SOURCES})
target_link_libraries(vslam_libraries ${third_party_lib})

# # feature_detection
# add_executable(feature_detection_node src/test/feature_detection.cpp)
# target_link_libraries(feature_detection_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

# calibrate intrinsic of stereo camera
add_executable(stereo_calibration_node src/test/calibrate_stereo.cpp)
target_link_libraries(stereo_calibration_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

# stereo deapth and disparity map
add_executable(stereo_depth_disparity_node src/test/stereo_disparity.cpp)
target_link_libraries(stereo_depth_disparity_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBRARIES})

# Visual SLAM node
add_executable(visual_slam_node src/visual_slam.cpp)
target_link_libraries(visual_slam_node vslam_libraries ${third_party_lib})