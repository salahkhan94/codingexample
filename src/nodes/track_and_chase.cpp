#include "ros/ros.h"
#include "nodelet/loader.h"
/*
This Node loads all the tracking and displaying nodelets 
pertaining to the mouse estimation 
*/
int main(int argc, char **argv) {

    ros::init(argc, argv, "track_and_chase_node");

    nodelet::Loader manager(false); // Don't bring up the manager ROS API
    nodelet::M_string remappings;
    nodelet::V_string my_argv(argv + 1, argv + argc);

//1
    std::string blob_detector_name = ros::this_node::getName() + "_blob_detector";
    manager.load(blob_detector_name, "track_and_chase/blob_detector", remappings, my_argv);
//2
    std::string display_output_name = ros::this_node::getName() + "_display_output";
    manager.load(display_output_name, "track_and_chase/display_output", remappings, my_argv);
//3
    std::string pose_estimator_name = ros::this_node::getName() + "_pose_estimator";
    manager.load(pose_estimator_name, "track_and_chase/pose_estimator", remappings, my_argv);
 
    ros::spin();
    return 0;
}