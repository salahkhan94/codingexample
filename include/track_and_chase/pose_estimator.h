#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <boost/version.hpp>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Pose2D.h>
#include <cmath>
#include <random>
#include <mutex>
#include "track_and_chase/kalman_filter.h"

using namespace std;

namespace track_and_chase {

class PoseEstimatorNodelet: public nodelet::Nodelet {
public:
    PoseEstimatorNodelet() {};
    ~PoseEstimatorNodelet() {};
private:
    //ROS communication 
    boost::shared_ptr<ros::NodeHandle> nh_;
    ros::Publisher pub_pos_, pub_img_;
    ros::Subscriber sub_img_, sub_pos_;
    
    //Stores estimated pose of the mouse
    geometry_msgs::Pose2D mouse_pose_;

    int n = 6; // Number of states X,Y, Vx, Vy, Ax, Ay
    int m = 2; // Number of measurements X Y
    int c = 2; // Number of control inputs
    double accel_sigma = 50; // standard deviation of acceleration pixel/s^2 
    int loop_rate_ = 30; //loop rate at which the prediction runs 
    double pi = 3.14159265359;
    bool init = false; 
    //wait until a single measurement is recieved to initialize 
    //the first pose X0

    //Kalman filter object
    KalmanFilter kf_;
    //Holds the current state of the kalman filter
    Eigen::VectorXd state_;

    //Initializes the Kalman Filter and and returns a KalmanFilter class object
    KalmanFilter getKalman();
    //initializes the subscribers and publishers and runs the prediction loop
    virtual void onInit();
    //measurement update callback, recieves pose from blob detection node
    void posecb(const geometry_msgs::Pose2D &pose_msg);
};

}