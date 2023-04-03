#include <boost/version.hpp>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include "opencv/highgui.h"
#include <geometry_msgs/Pose2D.h>
#include <cmath>

using namespace cv;
using namespace std;
namespace track_and_chase {

class BlobDetectorNodelet: public nodelet::Nodelet {
public:
    BlobDetectorNodelet(){};
    ~BlobDetectorNodelet(){};
private:
    // ROS communication
    boost::shared_ptr<ros::NodeHandle> nh_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::CameraSubscriber sub_;    //Subscribe to Video feed
    image_transport::CameraPublisher pub_img_; //Blob image publisher
    ros::Publisher pub_pos_;
    int queue_size_;


    vector<vector<Point>> contours_; //vector of contour boundary points
    vector<Vec4i> hierarchy_;
    

    virtual void onInit();

    void imageCb(const sensor_msgs::ImageConstPtr& image_msg,  //video feed image callback function
                const sensor_msgs::CameraInfoConstPtr& info_msg);
    
};

}