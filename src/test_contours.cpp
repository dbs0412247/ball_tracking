/* 
  - This code is heavily adapted from https://github.com/Itseez/opencv/blob/master/samples/cpp/camshiftdemo.cpp
  - The purpose is the implement a camshift tracker that runs within the ROS framework. 
  - Tracking is defined as the task of determining, given a bounding box of the object in the 
    first frame of an image sequence, the position and size (sometimes also orientation) of 
    the bounding box of the object in subsequent image frames. 
*/
#include <iostream>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/ocl.hpp>
#include <opencv2/ocl/ocl.hpp>
#include <opencv2/core/core.hpp> 
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

Mat image, target;

class ImageProcessor {

private:

// Private variable declarations
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  double canny_threshold1, canny_threshold2;
  SURF surf_;
 
public:
  ImageProcessor()
    : it_(nh_) {
    // initialization code for ROS
    image_pub_  = it_.advertise("/ps3_eye_processor/output", 1);
    image_sub_ = it_.subscribe("/ps3_eye/image_raw", 1, &ImageProcessor::imageCb, this);  

    // init Canny parameters   
/* 
    canny_threshold1 = 100.0;
    canny_threshold2 = 30.0; 
    int int_t1 = 100, int_t2 = 30;
    namedWindow("Canny");
    createTrackbar("Threshold1", "Canny", &int_t1, 200);
    createTrackbar("Threshold2", "Canny", &int_t2, 100);
*/
    // init SURF
    surf_.hessianThreshold = 0.5;
    surf_.nOctaves = 4;
    surf_.nOctaveLayers = 2;
    surf_.extended = true;
    surf_.upright = false;
    
  }

  ~ImageProcessor() {
//    destroyWindow("Canny");
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    // converting from ros image msg to cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    Mat gray; cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);

    // Canny edge detector    
    canny_threshold1 = (double)(getTrackbarPos("Threshold1", "Canny"));
    canny_threshold2 = (double)(getTrackbarPos("Threshold2", "Canny"));
    Mat canny; Canny(gray, canny, canny_threshold1, canny_threshold2);
//    imshow("Canny", canny);

    // SURF feature extractor
    vector<KeyPoint> keyPoint;
    Mat surfOutput; 
    surf_(gray, Mat(), keyPoint);
    drawKeypoints(
      cv_ptr->image, 
      keyPoint, 
      surfOutput, 
      Scalar(0, 255, 130),
      //DrawMatchesFlags::DRAW_RICH_KEYPOINTS
      DrawMatchesFlags::DEFAULT
    ); 
    

    // publish some image
    cv_bridge::CvImage cv_img;
    ros::Time time = ros::Time::now();
    cv_img.header.stamp = time;
    cv_img.encoding = enc::BGR8;
    //cv_img.image = cv_ptr->image;    
    cv_img.image = surfOutput;
    image_pub_.publish(cv_img.toImageMsg());
    waitKey(1);
  }
};

int main(int argc, char** argv)
{  
  ros::init(argc, argv, "test_contours");
  ImageProcessor contour_test;
  ros::spin();
  return 0;
}


