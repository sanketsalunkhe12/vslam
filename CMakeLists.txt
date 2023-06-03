cmake_minimum_required(VERSION 3.0.2)
project(vslam)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES vslam  
    CATKIN_DEPENDS roscpp
    # DEPENDS system_lib
)

include_directories(
    include
    data
    ${catkin_INCLUDE_DIRS}
)

# feature_detection
add_executable(feature_detection_node src/feature_detection.cpp)
target_link_libraries(feature_detection_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
