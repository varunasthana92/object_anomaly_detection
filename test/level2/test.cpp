#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cvTest");
  std::string filePath;  // Path of file which has information about measurements 
  const std::string paramName = "~filePath";
  bool check = ros::param::get(paramName, filePath);
  cv::Mat img;
  img =cv::imread(filePath,1);
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg; // >> message to be sent
  ros::NodeHandle nh;
  ros::Publisher pub_img = nh.advertise<sensor_msgs::Image>("/ur5/usbcam/image_raw", 1);
  std_msgs::Header header; // empty header
  header.seq = 1; // user defined counter
  header.stamp = ros::Time::now(); // time
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
  img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
  while(ros::ok()){
  pub_img.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
  }

  return 0;
}
