
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include <stdio.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <stdlib.h>

class Controller
{
public:
  
  Controller()
  {
    pub1 = n_.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    sub1 = n_.subscribe<geometry_msgs::Point>("near/goal", 10, &Controller::goal_callback, this);
    initialize();
    reset_goal_to_cur_pose();
    ros::Rate rate(20.0);
    while (n_.ok())
    {
      get_transform();
      update_pose();
      calculate_delta_angle();
      calculate_delta_dist();
      controller();
      publish_cmd_vel();
      ros::spinOnce();
      rate.sleep();
    }
  }
  
  void initialize()
  {
    cur_time = ros::Time::now();
    prev_time = ros::Time::now();
    simba_pose;
    goal_pose;
    e_dist = 0;
    e_angle = 0;
    e_angle_tolerance = 20 / 57.2958;
    e_dist_tolerance = 0.2;
};
   
  
  void reset_goal_to_cur_pose()
  {
    goal_pose.x = simba_pose.x;
    goal_pose.y = simba_pose.y;
    goal_pose.theta = simba_pose.theta;
  }
    
  void goal_callback(const geometry_msgs::Point::ConstPtr& data)
  {
    
    goal_pose.x = data->x;
    goal_pose.y = data->y;
    
  }
  
  void get_transform()
  {
      
      try
      {
        listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
    
  }
  
  void update_pose()
  {
     simba_pose.y = transform.getOrigin().y();
     simba_pose.x = transform.getOrigin().x();
     tf::Quaternion q = transform.getRotation(); 
     simba_pose.theta = tf::getYaw(q);
     //ROS_INFO("Theta is : %f",simba_pose.theta);
  }
//This is for calculating the delta angle  
  void calculate_delta_angle()
  {
     float f;
    f = atan2((goal_pose.y - simba_pose.y),( goal_pose.x - simba_pose.x) );
    if(f < 0)
     { 
       f = (2*3.1415926535897932384) + f;
     }
    e_angle = (f - simba_pose.theta);
    if(e_angle*57.2958 > 180)
    {
      e_angle = f - (2*3.1415926535897932384) - simba_pose.theta;
    }
    else if(e_angle*57.2958 < -180)
    {
      e_angle = f + (2*3.1415926535897932384) - simba_pose.theta;
    }
  }
  
  void calculate_delta_dist()
  {
    e_dist = sqrt(pow((simba_pose.x-goal_pose.x),2) + pow((simba_pose.y-goal_pose.y),2));
    ROS_INFO("e_dist is : %f",e_dist);
  }
  
  void publish_cmd_vel()
  {
    
    pub1.publish(cmd);
  }
  
  void controller()
  {
        ROS_INFO("e_dist_tolerance is : %f",e_dist_tolerance);
    if(fabsf(e_dist) > fabsf(e_dist_tolerance))
    {
 
      if(fabsf(e_angle) > fabsf(e_angle_tolerance))
      {
        if(e_angle > 0)
        { 
          
          cmd.angular.z = 1;
          cmd.linear.x = 0.0;
        }
        else
        {
         cmd.angular.z = -1;
          cmd.linear.x = 0.0;
        }
      }
      else
      {
        cmd.linear.x = 0.5;
        cmd.angular.z = 0;
      }
    }
    else
    {
    
      cmd.linear.x = 0;
      cmd.angular.z = 0;
    }
  }
      
private:
  ros::NodeHandle n_; 
  ros::Publisher pub1;
  ros::Subscriber sub1, sub2;
  ros::Duration elapsed_time;
  ros::Time cur_time;
  ros::Time prev_time;
  geometry_msgs::Pose2D simba_pose,goal_pose;
  tf::StampedTransform transform;
  float e_dist,e_angle;
  float e_angle_tolerance, e_dist_tolerance;
  geometry_msgs::Twist cmd;
  tf::TransformListener listener;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simba_simple_controller");
  Controller controllerObject;
  ros::spin();
  return 0;
}
