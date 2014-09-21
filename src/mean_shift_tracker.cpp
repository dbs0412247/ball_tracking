#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

Mat image, target;

bool backprojMode = false;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
Point origin;
Rect selection;
int vmin = 10, vmax = 256, smin = 30;

RotatedRect meanshift() {
  Rect trackWindow;
  int hsize = 16;
  float hranges[] = {0,180};
  const float* phranges = hranges;
  Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
  bool paused = false;

  if (!paused) {
    cvtColor(image, hsv, COLOR_BGR2HSV);
  }
}

class MeanShiftTracker {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  Mat target_, target_hue_, target_backproj_, target_hue_hist_;
  
public:
  MeanShiftTracker(Mat matImageTarget)
    : it_(nh_) {
    image_pub_ = it_.advertise("/meanshift_tracker/output", 1);
    image_sub_ = it_.subscribe("/ps3_eye/image_raw", 1, &MeanShiftTracker::imageCb, this);
    // Pre-process target image for tracking    
    int ch[] = {0, 0};
    int hsize = 16;
    float hranges[] = {0,180};
    const float* phranges = hranges;
    matImageTarget.copyTo(target_);
    // Extract hue channel
    Mat hsv;
    cvtColor(target_, hsv, CV_BGR2HSV);        
    mixChannels(&hsv, 1, &target_hue_, 1, ch, 1);
    // Calculate and normalize hue histogram
    calcHist(&target_hue_, 1, 0, 0, target_hue_hist_, 1, &hsize, &phranges);
    normalize(target_hue_hist_, target_hue_hist_, 0, 255, NORM_MINMAX);
    // Create a display image of the histogram
    Mat histimg = Mat::zeros(200, 320, CV_8UC3);
    int binW = histimg.cols / hsize;
    Mat buf(1, hsize, CV_8UC3);
    for( int i = 0; i < hsize; i++ )
      buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);
    cvtColor(buf, buf, COLOR_HSV2BGR);    
    for( int i = 0; i < hsize; i++ )
    {
      int val = saturate_cast<int>(target_hue_hist_.at<float>(i)*histimg.rows/255);
      rectangle( histimg, Point(i*binW,histimg.rows),
      Point((i+1)*binW,histimg.rows - val),
      Scalar(buf.at<Vec3b>(i)), -1, 8 );
    }
    // Display image of histogram
    imshow("Target Image Histogram", histimg);      
  }

  ~MeanShiftTracker() {
    destroyWindow("Target Image histogram");
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
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

    // begin meanshift
    
    

    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{  
  Mat target; 
  ros::init(argc, argv, "meanshift_tracker");
  // get target image 
  try { 
   target = imread("/home/adrian/Desktop/Image.jpg");
  } catch (cv::Exception cvex) {
    cout << "Could not read target image to track" << endl;
    cout << cvex.what() << endl;
    return -1;
  }
  MeanShiftTracker mst(target);
  ros::spin();
  return 0;
}


