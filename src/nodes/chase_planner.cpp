#include "track_and_chase/chase_planner.h"

/*
This Nodelet class implements a Position Controller and a Planner which 
publishes desired Robot velocities for it to chase down the fast moving mouse
*/
namespace track_and_chase {

ChasePlannerNode::ChasePlannerNode(const ros::NodeHandle &node_handle,
                                 const ros::NodeHandle &private_node_handle): 
    nh_(node_handle), pnh_(private_node_handle) {
    ros::param::param("~loop_rate_",loop_rate_, loop_rate_); //Set loop rate for posController
    this->onInit();
}

void ChasePlannerNode::onInit() { 

    //Initialize ROS communication
    sub_rpos_ = pnh_.subscribe("/robot/pose", 5, 
                            &ChasePlannerNode::rposecb, this);
    sub_mpos_ = pnh_.subscribe("/mouse/pose", 5, 
                            &ChasePlannerNode::mposecb, this);

    pub_rvel_cmd_ =  pnh_.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1);

    //Runs the posController func periodically
    periodic_timer_ = pnh_.createTimer(ros::Duration(1/loop_rate_),
                                       &ChasePlannerNode::posController,
                                       this);
}

//Robot pose callback
void ChasePlannerNode::rposecb(const geometry_msgs::Pose2D &pose_msg) {
    robot_pose_ = pose_msg;
}

//Mouse pose callback
void ChasePlannerNode::mposecb(const geometry_msgs::Pose2D &pose_msg) {
    mouse_pose_ = pose_msg;
}

//Function for calculating the angular error err_a 
//which is the angle that the robot has to rotate to face the mouse
void ChasePlannerNode::cal_angle_err() {

    float f;
    f = atan2((mouse_pose_.y - robot_pose_.y),( mouse_pose_.x - robot_pose_.x));
    if(f < 0) { 
        f = (2 * pi) + f;
    }
    err_a = (f - robot_pose_.theta);

    //Handle the jump as the err_a moves around 0 and 360
    // from an err_a of 1 degree the next error angle could be 359 degrees 
    // this would confuse the PID controller, hence the 359 degrees is converted to 
    //-1 degrees
    if(err_a > pi) {
        err_a = f - (2 * pi) - robot_pose_.theta;
    }
    else if(err_a< -pi)
    {
        err_a = f + (2 * pi) - robot_pose_.theta;
    }
}
//Calculate the distance between Robot and mouse
void ChasePlannerNode::cal_dist_err() {
    err_l = sqrt(pow((robot_pose_.x - mouse_pose_.x),2) + pow((robot_pose_.y - mouse_pose_.y),2));

    //Maintain a fixed tracking distance behind the mouse
    err_l -= tracking_dist_;
}

void ChasePlannerNode::posController(const ros::TimerEvent& event) {
    double kp_l = 1.0, ki_l = 0.1, kd_l = 0.000;
    double kp_a = 1.2, ki_a = 0.10, kd_a = 0.000;
    double errP_l, errI_l = 0.0, errD_l, prevError_l;
    double errP_a, errI_a = 0.0, errD_a, prevError_a;

    cal_dist_err(); //calculate distance error
    cal_angle_err(); //calculate angle error
    ROS_INFO_STREAM("Robot X : "<< robot_pose_.x);
    ROS_INFO_STREAM("Robot Y : "<< robot_pose_.y);
    ROS_INFO_STREAM("Angle error: "<< err_a);
    ROS_INFO_STREAM("Dist error : "<< err_l);

    // if robot facing a completely different 
    // direction reduce the angular error, first via pure rotation
    if(fabs(err_a) > e_angle_tolerance) { 

        ROS_INFO("Pure rotation command"); 
        errP_a = kp_a * err_a;
        errI_a += ki_a * err_a;
        errD_a = kd_a * (err_a - prevError_a);
        

        robot_vel_cmd.angular.z = errP_a + errI_a + errD_a;
        //Limit max commanded velocities
        robot_vel_cmd.angular.z = min(robot_vel_cmd.angular.z, max_angular_vel_);
        robot_vel_cmd.angular.z = max(robot_vel_cmd.angular.z, -max_angular_vel_);
        prevError_a = err_a;

        robot_vel_cmd.linear.x = 0.0;
        
    }
    //When the robot is more or less facing directly the mouse,
    //Drive forward
    else if(fabs(err_a) > e_angle_tolerance/5) {
        ROS_INFO("Pure translation command");
        errP_l = kp_l * err_l;
        errI_l += ki_l * err_l;
        errD_l = kd_l * (err_l - prevError_l);

        robot_vel_cmd.linear.x = errP_l + errI_l + errD_l;
        //Limit max commanded velocities
        robot_vel_cmd.linear.x = min(robot_vel_cmd.linear.x, max_linear_vel_);
        robot_vel_cmd.linear.x = max(robot_vel_cmd.linear.x, -max_linear_vel_);
        prevError_l = err_l;

        robot_vel_cmd.angular.z = 0;

    }
    //Robot generates translational and rotational velocities to guide it 
    //towards the mouse in a curved path
    else {
        ROS_INFO("translation and rotation command");
        errP_a = kp_a * err_a;
        errI_a += ki_a * err_a;
        errD_a = kd_a * (err_a - prevError_a);
        

        robot_vel_cmd.angular.z = errP_a + errI_a + errD_a;
        //Limit max commanded velocities
        robot_vel_cmd.angular.z = min(robot_vel_cmd.angular.z, max_angular_vel_);
        robot_vel_cmd.angular.z = max(robot_vel_cmd.angular.z, -max_angular_vel_);
        prevError_a = err_a;


        errP_l = kp_l * err_l;
        errI_l += ki_l * err_l;
        errD_l = kd_l * (err_l - prevError_l);

        robot_vel_cmd.linear.x = errP_l + errI_l + errD_l;
        //Limit max commanded velocities
        robot_vel_cmd.linear.x = min(robot_vel_cmd.linear.x, max_linear_vel_);
        robot_vel_cmd.linear.x = max(robot_vel_cmd.linear.x, -max_linear_vel_);
        prevError_l = err_l;

    }
    pub_rvel_cmd_.publish(robot_vel_cmd);
}
 
}
