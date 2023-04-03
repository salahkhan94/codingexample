#include "track_and_chase/chase_robot.h"
/*
Class defines a Differential 2D Robot Simulator
*/
namespace track_and_chase {

ChaseRobotNode::ChaseRobotNode(const ros::NodeHandle &node_handle,
                                 const ros::NodeHandle &private_node_handle): 
    nh_(node_handle), pnh_(private_node_handle) {
    
    this->onInit();
}

void ChaseRobotNode::onInit() {
    //Initialize ros communication nodes
    sub_twist_ = pnh_.subscribe("/robot/cmd_vel", 5, 
                            &ChaseRobotNode::cmdtwistcb, this);
    pub_pos_ =  pnh_.advertise<geometry_msgs::Pose2D>("/robot/pose", 1);
    pub_twist_ =  pnh_.advertise<geometry_msgs::Twist>("/robot/twist", 1);
    pub_accel_ =  pnh_.advertise<geometry_msgs::Accel>("/robot/accel", 1);

    //reset class member variables based on input from launch file
    ros::param::param("~max_angular_accel_",max_angular_accel_, max_angular_accel_);
    ros::param::param("~max_linear_accel_",max_linear_accel_, max_linear_accel_);
    ros::param::param("~loop_rate_",loop_rate_, loop_rate_);
    
    //Calls the velocity controller periodically
    periodic_timer_ = nh_.createTimer(ros::Duration(1/loop_rate_),
                                       &ChaseRobotNode::twistController,
                                       this);
    
}

void ChaseRobotNode::propagateState(double dt) {
    if((fabs(robot_vel_.angular.z)>0.0055 || fabs(robot_accel_.angular.z)>0.0055) && 
            (fabs(robot_vel_.linear.x)<0.055 && fabs(robot_accel_.linear.x)<0.55)) {
        
        //When Linear velocity and acceleration are almost 0 the robot undergoes pure
        //rotation
        ROS_INFO("Robot purely rotating");
        
        robot_pose_.theta += robot_vel_.angular.z*dt
                             + 0.5 * robot_accel_.angular.z * dt * dt;
        robot_vel_.angular.z += robot_accel_.angular.z * dt;

        //If angle greater than 360, take modulus
        robot_pose_.theta = fmod(robot_pose_.theta, 2 * pi);
        
        //Reset angles to be in the following range [0, 180] and [-180, 0]
        //instead of [0, 360]
        if(robot_pose_.theta > pi) robot_pose_.theta -= 2 * pi;
        if(robot_pose_.theta < -pi) robot_pose_.theta += 2 * pi;
    }

    else if ((fabs(robot_vel_.linear.x)>0.055 || fabs(robot_accel_.linear.x)>0.55) && 
                (fabs(robot_vel_.angular.z)<0.0055 && fabs(robot_accel_.angular.z)<0.0055) ){
        
        //When angular velocity and acceleration are almost 0, the robot undergoes pure 
        //translation
        ROS_INFO("Robot purely translating");
        
        robot_pose_.x += cos(robot_pose_.theta) * robot_vel_.linear.x * dt +
                        0.5 * cos(robot_pose_.theta) * dt * dt * robot_accel_.linear.x;
        robot_pose_.y += sin(robot_pose_.theta) * robot_vel_.linear.x * dt +
                        0.5 * sin(robot_pose_.theta) * dt * dt * robot_accel_.linear.x;
        robot_vel_.linear.x += robot_accel_.linear.x * dt;
    }

    else {
        //Motion model when the robot has significant angular velocity and translational velocity
        ROS_INFO("Robot rotating and translating");
        
        if(fabs(robot_vel_.angular.z)> 0.001) {
            //This check is done so as to not divide by a very small number
            double r = robot_vel_.linear.x / robot_vel_.angular.z;

            robot_pose_.x += r * cos(robot_pose_.theta) * sin(robot_vel_.angular.z * dt) +
                                r * sin(robot_pose_.theta) * cos(robot_vel_.angular.z * dt)
                                    - r * sin(robot_pose_.theta);

            robot_pose_.y += r * sin(robot_pose_.theta) * sin(robot_vel_.angular.z * dt) -
                                r * cos(robot_pose_.theta) * cos(robot_vel_.angular.z * dt)
                                    + r * cos(robot_pose_.theta); 
        }   

        robot_pose_.theta += robot_vel_.angular.z*dt
                                + 0.5 * robot_accel_.angular.z * dt * dt;     
        
        robot_vel_.angular.z += robot_accel_.angular.z * dt;

        robot_pose_.theta = fmod(robot_pose_.theta, 2 * pi);
        
        //Reset angles to be in the following range [0, 180] and [-180, 0]
        //instead of [0, 360]
        if(robot_pose_.theta > pi) robot_pose_.theta -= 2 * pi;
        if(robot_pose_.theta < -pi) robot_pose_.theta += 2 * pi;

        robot_vel_.linear.x += robot_accel_.linear.x * dt;
    }
}

//Recieve commanded velocity
void ChaseRobotNode::cmdtwistcb(const geometry_msgs::Twist &twist_msg) {
    robot_cmd_vel_ = twist_msg;
}

//Publish Robot state
void ChaseRobotNode::publishAll() {
    pub_accel_.publish(robot_accel_);
    pub_pos_.publish(robot_pose_);
    pub_twist_.publish(robot_vel_);
}
//PID Controller controlling the velocity of the robot via generated accel
void ChaseRobotNode::twistController(const ros::TimerEvent& event) {

    double kp_l = 1.0, ki_l = 0.1, kd_l = 0.000;
    double kp_a = 1.2, ki_a = 0.10, kd_a = 0.000;
    double err_l, errP_l, errI_l = 0.0, errD_l, prevError_l;
    double err_a, errP_a, errI_a = 0.0, errD_a, prevError_a;

    err_a = robot_cmd_vel_.angular.z - robot_vel_.angular.z;
    errP_a = kp_a * err_a;
    errI_a += ki_a * err_a;
    errD_a = kd_a * (err_a - prevError_a);

    robot_accel_.angular.z = errP_a + errI_a + errD_a;
    //enforce max acceleration
    robot_accel_.angular.z = min(robot_accel_.angular.z, max_angular_accel_);
    robot_accel_.angular.z = max(robot_accel_.angular.z, -max_angular_accel_);
    prevError_a = err_a;

    err_l = robot_cmd_vel_.linear.x - robot_vel_.linear.x;
    errP_l = kp_l * err_l;
    errI_l += ki_l * err_l;
    errD_l = kd_l * (err_l - prevError_l);

    robot_accel_.linear.x = errP_l + errI_l + errD_l;
    //enforce max_acceleration
    robot_accel_.linear.x = min(robot_accel_.linear.x, max_linear_accel_);
    robot_accel_.linear.x = max(robot_accel_.linear.x, -max_linear_accel_);
    prevError_l = err_l;

    //State Propagation
    propagateState(1/loop_rate_);
    publishAll();
}

}