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

# feature_detection
add_executable(feature_detection_node src/feature_detection.cpp)
target_link_libraries(feature_detection_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

# calibrate intrinsic of stereo camera
add_executable(stereo_calibration_node src/calibrate_stereo.cpp)
target_link_libraries(stereo_calibration_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
