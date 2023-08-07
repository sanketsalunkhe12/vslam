cmake_minimum_required(VERSION 3.0.2)
project(vslam)

# include(${CMAKE_CURRENT_SOURCE_DIR}/utils/DBoW2/CMakeLists.txt)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  message_filters
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(DBoW2 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES vslam DBoW2
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
            src/camera_calib.cpp
            src/feature_tracking.cpp
            src/visual_slam_manager.cpp
            src/keyframe.cpp
            )

set(DBoW2_SOURCES
    src/DBoW2/BowVector.cpp
    src/DBoW2/FBrief.cpp
    src/DBoW2/FORB.cpp 
    src/DBoW2/FSurf64.cpp
    src/DBoW2/QueryResults.cpp     
    src/DBoW2/FeatureVector.cpp
    src/DBoW2/ScoringObject.cpp)

add_library(DBoW2 SHARED ${DBoW2_SOURCES})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

add_library(vslam_libraries SHARED ${LIBRARY_SOURCES})
target_link_libraries(vslam_libraries ${third_party_lib})

# calibrate intrinsic of stereo camera
add_executable(stereo_calibration_node src/test/calibrate_stereo.cpp)
target_link_libraries(stereo_calibration_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

# stereo deapth and disparity map
add_executable(stereo_depth_disparity_node src/test/stereo_disparity.cpp)
target_link_libraries(stereo_depth_disparity_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBRARIES})

# Visual SLAM node
add_executable(visual_slam_node src/visual_slam.cpp)
target_link_libraries(visual_slam_node vslam_libraries ${third_party_lib})