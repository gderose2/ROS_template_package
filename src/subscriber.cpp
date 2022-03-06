// Include ROS base
#include <ros/ros.h>

// Include ROS messsage information
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

// Includes for working with images
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>

// Includes for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <template_package/SubscribeNodeDynCfgConfig.h>

// Include C/C++ standard libraries
#include <stdio.h>
#include <math.h>

// Define constants
#define CVWIN_PREVIEW "Target Object"



/******************************************************************************
*
* SubscribeNode class definition
*
******************************************************************************/
class SubscribeNode
{
public:
  SubscribeNode();
  ~SubscribeNode();

  void configCallback(template_package::SubscribeNodeDynCfgConfig &config,
		      uint32_t level);

private:
  /////////////////
  // Node Handler
  /////////////////
  ros::NodeHandle nh_;

  //////////////
  // Variables
  //////////////
  // Dynamic reconfigure variables
  int dyn_reconfig_int_ = 0;
  double dyn_reconfig_double_ = 0.0;

  // Threshold
  int thresh_ = 100;
  
  // ROS Parameters
  
  // ROS dynamic reconfigure server
  dynamic_reconfigure::Server<template_package::SubscribeNodeDynCfgConfig> server_;

  // ROS Time

  // ROS Topic Publishers
  image_transport::Publisher pub_img_;
  
  // ROS Topic Subscribers
  ros::Subscriber sub_int_;
  ros::Subscriber sub_str_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_img_;  
 
  
  ///////////////////////////////////
  // Supporting Function Prototypes
  ///////////////////////////////////
  void publish_img_message(void);
  void int_message_callback(const std_msgs::Int32::ConstPtr &msg);
  void str_message_callback(const std_msgs::String::ConstPtr &msg);
  void img_message_callback(const sensor_msgs::ImageConstPtr& msg);
};



/******************************************************************************
*
* SubscribeNode constructor
*
******************************************************************************/
SubscribeNode::SubscribeNode()
  : nh_{"~"}, it_{nh_}
{
  
    // Define publishers
    pub_img_ = it_.advertise("img_processed_msg", 10);

    // Define subscribers
    sub_int_ = nh_.subscribe("/publish_node/int_msg", 10,
			     &SubscribeNode::int_message_callback, this);
    sub_str_ = nh_.subscribe("/publish_node/str_msg", 10,
			     &SubscribeNode::str_message_callback, this);
    sub_img_ = it_.subscribe("/publish_node/img_msg", 10,
			     &SubscribeNode::img_message_callback, this);

    
    // Dynamic Reconfigure
    server_.setCallback(boost::bind(&SubscribeNode::configCallback,this,_1,_2));

    // Set ROS rate
    ros::Rate rate(50);

    // Report out node contstruction
    ROS_INFO_STREAM("subscribe_node running!");
}



/******************************************************************************
*
* SubscribeNode constructor
*
******************************************************************************/
SubscribeNode::~SubscribeNode()
{
    cv::destroyWindow(CVWIN_PREVIEW);
}


/******************************************************************************
*
* Dynamics Reconfigure
*
******************************************************************************/
void SubscribeNode::configCallback(template_package::SubscribeNodeDynCfgConfig
				 &config, uint32_t level)
{
  thresh_ = config.int_param;
  dyn_reconfig_double_ = config.double_param;
}
// End of configCallback



/******************************************************************************
*
* Support functions
*
******************************************************************************/

///////////////////////////////////////////////////////////////////////
// int_message_callback: Function to dispaly integer message received
///////////////////////////////////////////////////////////////////////
void SubscribeNode::int_message_callback(const std_msgs::Int32::ConstPtr &msg)
{
  // ROS_INFO("Recieved int = %d", msg->data);
  return;
}


//////////////////////////////////////////////////////////////////////
// str_message_callback: Function to display string message received
//////////////////////////////////////////////////////////////////////
void SubscribeNode::str_message_callback(const std_msgs::String::ConstPtr &msg)
{
  // ROS_INFO("Recieved str = %s", msg->data.c_str());
  return;
}


///////////////////////////////////////////////////////////////////////////
// img_message_callback: Function to place bounding box in input image
// Code based on OpenCV: Creating Bounding boxes and circles for contours 
///////////////////////////////////////////////////////////////////////////
void SubscribeNode::img_message_callback(const sensor_msgs::ImageConstPtr& msg)
{
 
  //Convert to cv image
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
  
  // Convert the source to grayscale
  cv::Mat src_gray;
  cv::cvtColor( cv_ptr->image, src_gray, cv::COLOR_BGR2GRAY );
  cv::blur( src_gray, src_gray, cv::Size(3,3) );

  cv::Mat canny_output;
  cv::Canny( src_gray, canny_output, thresh_, thresh_*2 );
  
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours( canny_output, contours,
		    cv::RETR_TREE,
		    cv::CHAIN_APPROX_SIMPLE );
  std::vector<std::vector<cv::Point>> contours_poly( contours.size() );
  std::vector<cv::Rect> boundRect( contours.size() );
  std::vector<cv::Point2f> centers( contours.size() );
  std::vector<float> radius( contours.size() );

  // Loop contours
  for( size_t i = 0; i < contours.size(); i++ )
    {
      cv::approxPolyDP( contours[i], contours_poly[i], 3, true );
      boundRect[i] = boundingRect( contours_poly[i] );
      cv::minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
    }
  
  cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );

  int x_sum = 0;
  int y_sum = 0;
  for( size_t i = 0; i< contours.size(); i++ )
    {
      cv::Scalar color = cv::Scalar(255,255,0);
      cv::drawContours( drawing, contours_poly, (int)i, color );
      cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2 );

      int xc = int ((boundRect[i].tl().x+boundRect[i].br().x)/2);
      int yc = int ((boundRect[i].tl().y+boundRect[i].br().y)/2);
      x_sum += xc;
      y_sum += yc;
      cv::circle(drawing,cv::Point(xc,yc), 5, color, 2 );
    }

  ROS_INFO("%d, %d", int (x_sum/(1.0*contours.size())),
	   int (y_sum/(1.0*contours.size())) );
  
  // Show preview window
  cv::imshow(CVWIN_PREVIEW, drawing);

  // Update GUI Window
  cv::waitKey(3);
  
  return;
  
}



/******************************************************************************
*
* Main function
*
******************************************************************************/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "subscribe_node");

    // Create a SubscribeNode object
    SubscribeNode sub_node_obj{};

    // Enter ROS loop
    ros::spin();
    return 0;
}
