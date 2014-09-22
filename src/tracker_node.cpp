#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

const int  NUMBER_OF_HISTOGRAM_BINS = 180;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ps3_eye/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/meanshift_tracker/output_video", 1);
  }

  ~ImageConverter()
  {
  }

  // A callback that is run whenever an image is received from ps3 eye
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    // initialization and variable decalarations
    int numHistogramBins = NUMBER_OF_HISTOGRAM_BINS;
    Mat frame, hsv, hue, backproj;
    cv_ptr->image.copyTo(frame);

    // convert image to hsv
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    // extract hue image
    int ch[] = {0, 0};
    hue.create(hsv.size(), hsv.depth());
    mixChannels(&hsv, 1, &hue, 1, ch, 1);
    
    // track the object via meanshift
    calcBackProj(&hue, 1, 0, hist, backproj, &phranges);
    


    // Draw an example circle on the video stream
    //  if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "meanshift_tracker");
  ImageConverter ic;
  ros::spin();
  return 0;
}
