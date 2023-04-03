#include <track_and_chase/display_output.h>
/*
This nodelet accesses data from robot and mouse and displays them
along with their orientation/heading on the blob detected image
*/

using namespace std;

namespace track_and_chase {

void DisplayNodelet::onInit() {
    ros::NodeHandle& nh_ = getNodeHandle();
    //Initialize subscribers and publishers
    sub_img_ = nh_.subscribe("/blob_detected/image_raw", 
                            5, &DisplayNodelet::imagecb, this);
    sub_mpos_ = nh_.subscribe("/mouse/pose", 5, 
                            &DisplayNodelet::mposecb, this);
    sub_rpos_ = nh_.subscribe("/robot/pose", 5, 
                            &DisplayNodelet::rposecb, this);

    //Final Output image
    pub_img_ = nh_.advertise<sensor_msgs::Image>("/output/image_raw", 1);
    //wait until at least one image is recieved
    while(!init && ros::ok());

}
//Blob detected image cb
void DisplayNodelet::imagecb(const sensor_msgs::ImageConstPtr& image_msg) {
    
    try {
        //Lock the imgPtr while copying a new message
        img_mutex.lock();
        imgPtr = cv_bridge::toCvCopy(image_msg);
        if(!init) init = true;
        img_mutex.unlock();
    }

    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void DisplayNodelet::mposecb(const geometry_msgs::Pose2D &pose_msg) {
    mouse_pose_ = pose_msg; //update mouse_pose
    if(img_mutex.try_lock()) {
        double l = 60; //length of arrow in pixels
        cv::Point P2;
        //calculate 2d point of the arrow head
        P2.x = mouse_pose_.x + l*cos(mouse_pose_.theta);
        P2.y = mouse_pose_.y + l*sin(mouse_pose_.theta);
        //Draw arrow denoting mouse heading direction
        cv::arrowedLine(imgPtr->image, cv::Point(mouse_pose_.x, mouse_pose_.y), P2, cv::Scalar(255,0,0));
        //Draw a circle representing position estimate of the mouse from KF
        cv::circle(imgPtr->image, cv::Point(mouse_pose_.x, mouse_pose_.y), 40, cv::Scalar(0,255,255));
        
        //Calculate 2d point of the arrow head denoting heading of the robot
        P2.x = robot_pose_.x + l*cos(robot_pose_.theta);
        P2.y = robot_pose_.y + l*sin(robot_pose_.theta);
        //draw arrow to denote yaw orientation of the robot
        cv::arrowedLine(imgPtr->image, cv::Point(robot_pose_.x, robot_pose_.y), P2, cv::Scalar(255,0,0));
        //draw circle to represent current position of the robot
        cv::circle(imgPtr->image, cv::Point(robot_pose_.x, robot_pose_.y), 40, cv::Scalar(255,255,255));
        
        // publish the newly drawn image
        pub_img_.publish(imgPtr->toImageMsg());
        img_mutex.unlock();
    }
}

//Pose callback from robot
void DisplayNodelet::rposecb(const geometry_msgs::Pose2D &pose_msg) {
    robot_pose_ = pose_msg;
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(track_and_chase::DisplayNodelet, nodelet::Nodelet)
