What: OpenCV's camshiftdemo.cpp modified to support ROS messages. 

How to run: 

1) Modify ps3_eye.launch, change param "video_device" to suit your environment

2) roslaunch ps3_eye.launch

3) rosrun ball_tracking cam_shift_node

4) Adjust sliders the modify the minimum s, minimum and maximum v of the source   image (as in hsv format)

5) Left click and drag on source image to provide the tracker the initial information

Topics published: 
- Image Transport Messages: 
  /meanshift_tracker/frame 
  /meanshift_tracker/histogram
- Custom ROS messages:
  /meanshift_tracker/camshift_output (rosmsg show CamshiftOutput.msg)

