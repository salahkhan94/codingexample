#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <boost/version.hpp>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <geometry_msgs/Pose2D.h>
#include <cmath>
#include <random>
#include <mutex>

using namespace std;

namespace track_and_chase {

class DisplayNodelet: public nodelet::Nodelet {
public:
    DisplayNodelet() {};
    ~DisplayNodelet() {};
private:
    //ROS Communication
    boost::shared_ptr<ros::NodeHandle> nh_;
    ros::Subscriber sub_img_, sub_mpos_, sub_rpos_;
    ros::Publisher pub_img_;

    geometry_msgs::Pose2D robot_pose_, mouse_pose_;
    //holds the image with the blob detectioon
    cv_bridge::CvImagePtr imgPtr;

    //For sequential access to imgPtr
    mutex img_mutex;
    
    //denotes if imgPtr has been updated atleast once before 
    //attempting to display position of Robot and Mouse
    bool init = false;
    
    //Subscribes to blob detection topic 
    void imagecb(const sensor_msgs::ImageConstPtr &image_msg);
    // mouse position cb
    void mposecb(const geometry_msgs::Pose2D &pose_msg);
    // robot positon cb
    void rposecb(const geometry_msgs::Pose2D &pose_msg);
    //initialize subscribers and publishers
    virtual void onInit();
};


}