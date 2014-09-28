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

class MeanShiftTracker {

private:

// Private variable declarations
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  bool backprojMode, selectObject, showHist, paused;
  int trackObject, vmin, vmax, smin, hsize;
  Point origin;
  Rect selection, trackWindow;
  Mat hsv, hue, mask, hist, backproj, histimg;
  vector<Mat> hsv_split;

// private function declarations

  // mouse callback handler
  void onMouse( int event, int x, int y ) {
    if( selectObject ) {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);

        selection &= Rect(0, 0, image.cols, image.rows);
    }

    switch( event )  {
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

  // helper function to call the actual callback handler
  static void onMouse (int event, int x, int y, int, void* this_) {   
    static_cast<MeanShiftTracker*>(this_)->onMouse(event, x, y);
  }


  // createHistogramImage is a wrapper for the steps and functions to 
  // generate the image of a histogram
  Mat createHueHistogramImage(Mat hist_, int hsize_) {
    // Create a display image of the histogram
    Mat histimg = Mat::zeros(200, 320, CV_8UC3);
    int binW = histimg.cols / hsize;
    Mat buf(1, hsize_, CV_8UC3);
    for( int i = 0; i < hsize_; i++ )
      buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize_), 255, 255);
    cvtColor(buf, buf, COLOR_HSV2BGR);    
    for( int i = 0; i < hsize_; i++ )
    {
      int val = 
        saturate_cast<int>(hist_.at<float>(i)*histimg.rows/255);
      rectangle( histimg, Point(i*binW,histimg.rows),
      Point((i+1)*binW,histimg.rows - val),
      Scalar(buf.at<Vec3b>(i)), -1, 8 );
    }
    return histimg; 
  }
  
public:
  MeanShiftTracker()
    : it_(nh_) {
    // initialization code for ROS
    image_pub_ = it_.advertise("/meanshift_tracker/output", 1);
    image_sub_ = it_.subscribe("/ps3_eye/image_raw", 1, &MeanShiftTracker::imageCb, this);
    // initialization code for camshift tracking
    backprojMode = false;
    selectObject = false;
    paused = false;
    trackObject = 0;
    showHist = true;
    vmin = 10, vmax = 256, smin = 30; 
    hsize = 36;
  
    // Creating the window before setmousecallback and createTrackbar
    // otherwise it doesn't work 
    namedWindow("Current Frame");
    setMouseCallback("Current Frame", &onMouse, this);
    createTrackbar( "Vmin", "Current Frame", &vmin, 256, 0 );
    createTrackbar( "Vmax", "Current Frame", &vmax, 256, 0 );
    createTrackbar( "Smin", "Current Frame", &smin, 256, 0 );
   }

  ~MeanShiftTracker() {
    destroyWindow("Current Frame");
    destroyWindow("Current Hue Histogram");
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    // some declarations and initialization
    float hranges[] = {0,180};
    const float* phranges = hranges;

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
    cv_ptr->image.copyTo(image);
    
    if (!paused) {      

      cvtColor(image, hsv, CV_BGR2HSV);

      if (trackObject) {
        int _vmin = vmin, _vmax = vmax;
        inRange(
          hsv, 
          Scalar(0, smin, MIN(_vmin,_vmax)),
          Scalar(180, 256, MAX(_vmin, _vmax)), 
          mask
        ); 

        // extract hue channel from image
        split(hsv, hsv_split);
        hue = hsv_split[0];
       
        if( trackObject < 0 ) {
          Mat roi(hue, selection), maskroi(mask, selection);
          calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
          normalize(hist, hist, 0, 255, NORM_MINMAX);
          trackWindow = selection;
          trackObject = 1;
          histimg = createHueHistogramImage(hist, hsize);
        } // end if (trackObject < 0)

        calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
        backproj &= mask;
        RotatedRect trackBox = CamShift(
          backproj, trackWindow,
          TermCriteria( TermCriteria::EPS | TermCriteria::COUNT, 10, 1 )
        );
        if( trackWindow.area() <= 1 ) {
          int 
            cols = backproj.cols, 
            rows = backproj.rows, 
            r = (MIN(cols, rows) + 5)/6;
          trackWindow = Rect(
            trackWindow.x - r, 
            trackWindow.y - r,
            trackWindow.x + r, 
            trackWindow.y + r) 
            & Rect(0, 0, cols, rows);
        }
        if( backprojMode )
          cvtColor( backproj, image, COLOR_GRAY2BGR );
        ellipse( image, trackBox, Scalar(0,0,255), 3, CV_AA );

      } // end if (trackObject)
    } // end if (!paused)
    else if (trackObject < 0) 
      paused = false;

    
    if( selectObject && selection.width > 0 && selection.height > 0 ) {
      Mat roi(image, selection);
      bitwise_not(roi, roi);
    }

    imshow( "Current Frame", image );
    if (!histimg.empty()) 
      imshow( "Current Hue Histogram", histimg );

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


