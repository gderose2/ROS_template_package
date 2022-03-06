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
#include <template_package/PublishNodeDynCfgConfig.h>

// Include C/C++ standard libraries
#include <stdio.h>
#include <math.h>

// Define constants
#define CVWIN_PREVIEW "Moving Object"



/******************************************************************************
*
* PublishNode class definition
*
******************************************************************************/
class PublishNode
{
public:
  PublishNode();
  ~PublishNode();

  void configCallback(template_package::PublishNodeDynCfgConfig &config,
		      uint32_t level);

private:
  /////////////////
  // Node Handler
  /////////////////
  ros::NodeHandle nh_;

  //////////////
  // Variables
  //////////////
  // Counter
  int counter_ = 0;

  // Dynamic reconfigure variables
  int dyn_reconfig_int_ = 0;
  double dyn_reconfig_double_ = 0.0;
  
  // ROS Parameters
  
  // ROS dynamic reconfigure server
  dynamic_reconfigure::Server<template_package::PublishNodeDynCfgConfig> server_;

  // ROS Time

  // ROS Topic Publishers
  image_transport::Publisher pub_img_;
  image_transport::ImageTransport it_;
  ros::Publisher pub_int_;
  ros::Publisher pub_str_;

  // ROS Topic Subscribers


  ///////////////////////////////////
  // Supporting Function Prototypes
  ///////////////////////////////////
  void publish_int_message(void);
  void publish_str_message(void);
  void publish_img_message(void);
};



/******************************************************************************
*
* PublishNode constructor
*
******************************************************************************/
PublishNode::PublishNode()
  : nh_{"~"}, it_{nh_}
{
  
    // Define publishers
    pub_int_ = nh_.advertise<std_msgs::Int32>("int_msg", 10);
    pub_str_ = nh_.advertise<std_msgs::String>("str_msg", 10);
    pub_img_ = it_.advertise("img_msg", 10);

    // Define subscribers
  
    // Dynamic Reconfigure
    server_.setCallback(boost::bind(&PublishNode::configCallback,this,_1,_2));

    // Set ROS rate
    ros::Rate rate(50);

    // Report out node contstruction
    ROS_INFO_STREAM("publish_node running!");

    // Start ROS loop
    while(ros::ok()) {

      // Call publishers
      publish_int_message();
      publish_str_message();
      publish_img_message();
	
      // Control time step
      rate.sleep();
    }
  
}



/******************************************************************************
*
* PublishNode constructor
*
******************************************************************************/
PublishNode::~PublishNode()
{
    cv::destroyWindow(CVWIN_PREVIEW);
}


/******************************************************************************
*
* Dynamics Reconfigure
*
******************************************************************************/
void PublishNode::configCallback(template_package::PublishNodeDynCfgConfig
				 &config, uint32_t level)
{
  dyn_reconfig_int_ = config.int_param;
  dyn_reconfig_double_ = config.double_param;
}



/******************************************************************************
*
* Support functions
*
******************************************************************************/

////////////////////////////////////////////////////////////////
// publish_int_message: Fucntion to publish an integer message
////////////////////////////////////////////////////////////////
void PublishNode::publish_int_message()
{

  // Define messages
  std_msgs::Int32 msg;
  msg.data = counter_;
    
  // Publish message
  pub_int_.publish(msg);

  // Increment integer counter
  counter_++;

}
// End of publish_int_message



/////////////////////////////////////////////////////////////////
// publish_str_message: Function to publishing a string message
/////////////////////////////////////////////////////////////////
void PublishNode::publish_str_message()
{
  char buf[100];

  // Build string
  int n = sprintf(buf, "Counter = %d", counter_ );
  
  // Define messages
  std_msgs::String msg;
  msg.data = buf;
    
  // Publish message
  pub_str_.publish(msg);

}
// End of publish_str_message



/////////////////////////////////////////////////////////////////
// publish_img_message: Function to publishing an image message
/////////////////////////////////////////////////////////////////
void PublishNode::publish_img_message()
{
  int rows = 480;
  int cols = 640;
  int radius = 50;
  
  // Create blank image
  cv::Mat img(rows, cols, CV_8UC3, cv::Scalar(0, 0, 0));

  // Set the circle movement freqency
  double freq = 0.1; // Hz
  
  // Get current time
  double t =ros::Time::now().toSec();
  int xc = cols/2 + int (sin(2.0*CV_PI*freq*t)*cols/4);
  int yc = rows/2 + int (sin(2.0*CV_PI*freq/1.5*t)*rows/4);

  // Draw a circle at the current location
  cv::circle( img,
	      cv::Point(xc, yc),
	      radius,
	      cv::Scalar( 0, 0, 255 ),
	      cv::FILLED,
	      cv::LINE_8 );

  cv::circle( img,
	      cv::Point(xc, yc+radius),
	      radius-25,
	      cv::Scalar( 0, 0, 255 ),
	      cv::FILLED,
	      cv::LINE_8 );

  cv::circle( img,
	      cv::Point(xc, yc-radius),
	      radius-25,
	      cv::Scalar( 0, 0, 255 ),
	      cv::FILLED,
	      cv::LINE_8 );

  // Publish message
  sensor_msgs::ImagePtr msg;
  msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
			   img).toImageMsg();
  pub_img_.publish(msg);

  // Show preview window
  cv::imshow(CVWIN_PREVIEW, img);

  // Update GUI Window
  cv::waitKey(3);
}
// End of publish_str_message




/******************************************************************************
*
* Main function
*
******************************************************************************/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_node");

    // Create a PublishNode object
    PublishNode pub_node_obj{};

    // Enter ROS loop
    ros::spin();
    return 0;
}
