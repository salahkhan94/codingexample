#include "ros/ros.h"
#include "track_and_chase/chase_robot.h"
#include "track_and_chase/chase_planner.h"
/*
An executable of this node is generated and called using the launch file 
launches the robot simulator and the robot path planner 
*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "chase_robot_node");
    
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    //Robot simulator
    track_and_chase::ChaseRobotNode node1(nh, nh_private);
    //Robot path planner and position control node
    track_and_chase::ChasePlannerNode node2(nh, nh_private);

    ROS_INFO("Chase Robot multithreaded node initialized.");
    
    ros::AsyncSpinner s(8);  // Use 4 threads
    s.start();
    
    ros::waitForShutdown();
    
    return 0;

}