#ifndef VISUAL_SLAM_MANAGER_HPP
#define VISUAL_SLAM_MANAGER_HPP

#include "ros/ros.h"

#include "DBoW2/TemplatedVocabulary.h"
#include "DBoW2/FORB.h"


class VisualSlamManager
{
    public:
        VisualSlamManager();


    private:

        // DBoW object: DBoW ORB Vocalbulary database        
        // ORBVocabulary = new DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>();
  

        // keyframe database object: Keyframe database using keyframes


        // atlas map object: Atlas map structure which will store pointers to each keyframe 
        //                   and map-points


        // tracking object: tracker which will receieve new frame and compute camera pose
        //                  Decide to insert that frame as keyframe or not
        //                  Creating new map-points and 
        //                  perform relocalization in case of lost tracking


        // local mapper object: create a local map and perform local bundle adjustment


        // Loop closing object: search for each new keyframe which have correspodance
        //                      with keyframe database and perform full loop closure using 
        //                      pose graph optimization



        // viewer object: visualize map and robot pose. We are using rviz instead of pangolin


        // frame drawer and map drawer ?


        // multi-threading for local mapping, loop closure, and viewer.


        // Mutex parameters for data management between threads and some extra variables






};


#endif  