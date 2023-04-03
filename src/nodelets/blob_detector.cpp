#include "track_and_chase/blob_detector.h"

using namespace cv;
using namespace std;

namespace track_and_chase {

void BlobDetectorNodelet::onInit() {
    ros::NodeHandle& nh_ = getNodeHandle();
    //ROS communication setup to exchange image info
    it_.reset(new image_transport::ImageTransport(nh_));
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());

    sub_ = it_->subscribeCamera("/videofile/image_raw", queue_size_, &BlobDetectorNodelet::imageCb, this, hints);
    pub_img_ = it_->advertiseCamera("/blob_detected/image_raw", 1);

    pub_pos_ =  nh_.advertise<geometry_msgs::Pose2D>("/blob_detected/pose", 1);
}
//Video image callback
void BlobDetectorNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                                  const sensor_msgs::CameraInfoConstPtr& info_msg) {
    int width = image_msg->width;
    int height = image_msg->height;
    cv_bridge::CvImagePtr imgPtr;
    try {
        //Copy the image into a local class member
        imgPtr = cv_bridge::toCvCopy(image_msg);
    }

    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //Generate a thresolded image
    cv::Mat thresholded;
    cv::cvtColor(imgPtr->image, thresholded, CV_RGB2GRAY);
    const int thresholdLevel = 130;
    cv::threshold(thresholded, thresholded, thresholdLevel, 255, CV_THRESH_BINARY);
    
    //Find contours
    findContours(thresholded, contours_, hierarchy_, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    if(contours_.size() <= 0) {
        ROS_WARN("mouse not found found");
        return;
    }

    Moments mu = moments(contours_[0], false);
    //Find the centroid on the contour boundary of the blob
    geometry_msgs::Pose2D msg;
    msg.x =  mu.m10 / mu.m00;
    msg.y = mu.m01 / mu.m00;
    msg.theta =  atan((float) msg.y/msg.x) * (180/3.14159);


    //Draw a circle signifying the detection
    cv::circle(imgPtr->image, cv::Point(msg.x, msg.y), 50, 255);
    //Publish the new image and the position of the mouse
    pub_img_.publish(imgPtr->toImageMsg(), info_msg);
    pub_pos_.publish(msg);
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(track_and_chase::BlobDetectorNodelet, nodelet::Nodelet)