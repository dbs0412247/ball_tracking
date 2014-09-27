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
#include <opencv2/core/core.hpp> 
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


static void onMouse( int event, int x, int y, int, void* )
{
    if( selectObject )
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);

        selection &= Rect(0, 0, image.cols, image.rows);
    }

    switch( event )
    {
    case EVENT_LBUTTONDOWN:
        origin = Point(x,y);
        selection = Rect(x,y,0,0);
        selectObject = true;
        break;
    case EVENT_LBUTTONUP:
        selectObject = false;
        if( selection.width > 0 && selection.height > 0 )
            trackObject = -1;
        break;
    }
}

class MeanShiftTracker {

private:

// Private variable declarations
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
 
// private function declarations

  // createHistogramImage is a wrapper for the steps and functions to 
  // generate the image of a histogram
  Mat createHueHistogramImage(Mat hue, int hsize) {
    // Pre-process target image for tracking    
    int ch[] = {0, 0};    
    float hranges[] = {0,180};
    const float* phranges = hranges;
    // Calculate and normalize hue histogram
    Mat hist;
    calcHist(&hue, 1, 0, Mat(), hist, 1, &hsize, &phranges);
    normalize(hist, hist, 0, 255, NORM_MINMAX);
    // Create a display image of the histogram
    Mat histimg = Mat::zeros(200, 320, CV_8UC3);
    int binW = histimg.cols / hsize;
    Mat buf(1, hsize, CV_8UC3);
    for( int i = 0; i < hsize; i++ )
      buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);
    cvtColor(buf, buf, COLOR_HSV2BGR);    
    for( int i = 0; i < hsize; i++ )
    {
      int val = 
        saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);
      rectangle( histimg, Point(i*binW,histimg.rows),
      Point((i+1)*binW,histimg.rows - val),
      Scalar(buf.at<Vec3b>(i)), -1, 8 );
    }
    return histimg; 
  }

  
public:
  MeanShiftTracker()
    : it_(nh_) {
    image_pub_ = it_.advertise("/meanshift_tracker/output", 1);
    image_sub_ = it_.subscribe("/ps3_eye/image_raw", 1, &MeanShiftTracker::imageCb, this);
   }

  ~MeanShiftTracker() {
    destroyWindow("Current Frame");
    destroyWindow("Current Hue Histogram");
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
    Mat hsv; vector<Mat> hsv_split;
    cvtColor(cv_ptr->image, hsv, CV_BGR2HSV);
    split(hsv, hsv_split);
    Mat histimg = createHueHistogramImage(hsv_split[0], 36);
    imshow ("Current Frame", cv_ptr->image);
    imshow ("Current Hue Histogram", histimg);
    cv_ptr->image = histimg;

    // publish some image
    image_pub_.publish(cv_ptr->toImageMsg());
    waitKey(1);
  }
};

int main(int argc, char** argv)
{  
  ros::init(argc, argv, "meanshift_tracker");
  MeanShiftTracker mst;
  ros::spin();
  return 0;
}


