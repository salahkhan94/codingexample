#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <cmath>

using namespace std;
namespace track_and_chase {

class ChaseRobotNode {
public:
    ChaseRobotNode(const ros::NodeHandle &node_handle,
                                 const ros::NodeHandle &private_node_handle);
    ~ChaseRobotNode() {};
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_; //Private nodehandle

    //ROS Communication
    ros::Publisher pub_pos_, pub_twist_, pub_accel_;
    ros::Subscriber sub_twist_;

    //Robot Pose 
    geometry_msgs::Pose2D robot_pose_;
    geometry_msgs::Twist robot_vel_; //Current Robot body velocity
    geometry_msgs::Accel robot_accel_;//Current Robot body acceleration
    geometry_msgs::Twist robot_cmd_vel_; //holds the velocity commands recieved

    ros::Timer periodic_timer_; //Used to run the velocity controller

    int size_ = 40; //Robot size in Pixels
    double max_linear_accel_ = 100; //Maximum linear Acceleration
    double max_angular_accel_ = 100; //Maximum angular accleration
 
    float loop_rate_ = 30; //Loop rate at which the velocity controller function runs
    double pi = 3.14159265359;

    //Robot's velocity PID controller which takes a command velocity as input
    //And controls the robot's acceleration to achieve that velocity
    void twistController(const ros::TimerEvent& event);

    //Velocity command callback, updates robot_cmd_vel_
    void cmdtwistcb(const geometry_msgs::Twist &twist_msg);
    
    //State propagation function based on forward kinematic equations
    void propagateState(double dt);
    
    //Initialized Ros subscribers and publishers and runs a periodically called functions
    void onInit();
    //Publish Robot states
    void publishAll();
};

}