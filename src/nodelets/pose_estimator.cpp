#include "track_and_chase/pose_estimator.h"

using namespace std;
/*
The poseEstimatorNodelet probabilistically combines motions model and
sensor information about the position of blob to generate a 
filtered pose output 
*/
namespace track_and_chase {


void PoseEstimatorNodelet::onInit() {
    ros::NodeHandle& nh_ = getNodeHandle();
    ros::param::param("~loop_rate_",loop_rate_, loop_rate_);
    sub_pos_ = nh_.subscribe("/blob_detected/pose", 5, 
                            &PoseEstimatorNodelet::posecb, this);
    
    //Output of the estimator published
    pub_pos_ =  nh_.advertise<geometry_msgs::Pose2D>("/mouse/pose", 1);
    //Define the Covariance, State Matrix, Measurement Matrix and 
    //return the KF object
    kf_ = getKalman();

    //Wait for first measurement to initialize the state
    while(!init && ros::ok());

    //initialize the KF state
    kf_.init(state_);
    //input signal
    Eigen::VectorXd u(2);
//    std::default_random_engine generator;
//    std::normal_distribution<double> dist(0, accel_sigma);
    
    ros::Rate loop_rate(loop_rate_);
    
    while(ros::ok()) {
        //There is 0 Input to the model
        u(0) = 0.0; // dist(generator);
        u(1) = 0.0; // dist(generator);
        // Next State Prediction
        kf_.predict(u);
        state_ = kf_.state();
  
        //Limit the position to the boundary of the image
        state_(0) = max(0.0, state_(0));
        state_(0) = min(1280.0, state_(0));
        state_(1) = max(0.0, state_(1));
        state_(1) = min(720.0, state_(1));
  
        mouse_pose_.x = state_(0); //X coordinate of the mouse
        mouse_pose_.y = state_(1); //Y coordinate of the mouse
        //Heading angle : taninverse of Vy/Vx
        mouse_pose_.theta = atan2(state_(3),state_(2));

        //reset the orientation from [0, 360] to [0, 180] & [-180, 0]
        if(mouse_pose_.theta > pi) mouse_pose_.theta -= 2 * pi;
        if(mouse_pose_.theta < -pi) mouse_pose_.theta += 2 * pi;

        //publish poses
        pub_pos_.publish(mouse_pose_);
        loop_rate.sleep();
    }
}



KalmanFilter PoseEstimatorNodelet::getKalman() {

    Eigen::MatrixXd A(n, n); // System dynamics matrix
    Eigen::MatrixXd B(n, c); // Input control matrix
    Eigen::MatrixXd C(m, n); // Output matrix
    Eigen::MatrixXd Q(n, n); // Process noise covariance
    Eigen::MatrixXd R(m, m); // Measurement noise covariance
    Eigen::MatrixXd P(n, n); // Estimate error covariance

    //State : X,Y, Vx, Vy, Ax, Ay
    double dt = 1.0/loop_rate_;
    //State Transition Matrix
    A << 1,  0, dt,  0, dt*dt*0.5,      0,
         0,  1,  0, dt,     0,  dt*dt*0.5,
         0,  0,  1,  0,     dt,         0,
         0,  0,  0,  1,      0,        dt,
         0,  0,  0,  0,      1,         0,
         0,  0,  0,  0,      0,         1;

    //unimportant as there is 0 input
    B << dt*dt*0.5,           0,   
                 0,   dt*dt*0.5,
                dt,           0,
                 0,          dt,
                 0,          0,
                 0,          0;

    //Measurement Matrix 
    C << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0;
    
    //Process Noise covariance Matrix
    Q << 0.1,       0,   0,     0,  0,  0,
         0,       0.1,   0,     0,  0,  0,
         0,         0,   0.05,   0,  0,  0,
         0,         0,   0,   0.05,  0,  0,
         0,         0,   0,     0,  10,  0,
         0,         0,   0,     0,  0,  10;

    //Q *= accel_sigma*accel_sigma;

    P <<  10, 0,  0, 0,  0, 0,
            0, 10, 0, 0, 0, 0,
            0, 0, 10, 0, 0, 0,
            0, 0,  0, 10,0, 0,
            0, 0,  0,  0, 1, 0,
            0, 0,  0,  0, 0, 1;
    
    //Measurement Noise Covariance Matrix
    R << 0.005, 0,
           0, 0.005;
    
    KalmanFilter kf1(A, B, C, Q, R, P);
    return kf1;
}

//Recieve measurement
void PoseEstimatorNodelet::posecb(const geometry_msgs::Pose2D &pose_msg) {
    if(!init) {

        state_.resize(n);
        state_.setZero();

        state_(0) = pose_msg.x;
        state_(1) = pose_msg.y;
        //make true when first measurement update arrives to initialize state vector
        init = true;
        return;
    }
    Eigen::VectorXd y(m);
    y << pose_msg.x, pose_msg.y;
    //measurement update
    kf_.update(y);
    
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(track_and_chase::PoseEstimatorNodelet, nodelet::Nodelet)