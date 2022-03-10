#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np

from std_msgs.msg import Int32
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from dynamic_reconfigure.server import Server
from template_package.cfg import PublishNodeDynCfgConfig

ACTIVE_WINDOWS = []

# PublishNode class definition
class PublishNode():
    def __init__(self):
        """Template package publisher node class"""

        # Variables
        self.counter_ = 0
        
        # Dynamic reconfigure variables
        self.dyn_config = []
        self.dyn_reconfig_int = 0
        self.dyn_reconfig_double = 0.0


        # ROS Parameters
  
        # ROS dynamic reconfigure server
        self.srv = Server(PublishNodeDynCfgConfig, self.dyn_reconfig_callback)
        
        # ROS Time

        # ROS Topic Publisher
        self.pub_int_ = rospy.Publisher('int_msg', Int32, queue_size=10)
        self.pub_str_ = rospy.Publisher('str_msg', String, queue_size=10)
        self.pub_img_ = rospy.Publisher('img_msg', Image, queue_size=10)

        # ROS Topic Subscribers

        # Define ROS rate
        self.rate = rospy.Rate(50)

        # Set up CV Bridge
        self.bridge = CvBridge()
        
        # Report out node construction
        rospy.loginfo('publish_node running!')
        
        # Start ROS loop
        while not rospy.is_shutdown():
            
            #  Call publishers
            self.publish_int_message()
            self.publish_str_message()
            self.publish_img_message()
            
            # Control time step
            self.rate.sleep()
            
        return

    
    ################################
    # Dynamic Reconfigure callback
    ################################
    def dyn_reconfig_callback(self, config, level):
        self.dyn_reconfig_int = config['int_param']
        self.dyn_reconfig_double = config['double_param']
        return config

    
    ############################
    # Publish integer messsage
    ############################
    def publish_int_message(self):
        # Build message
        msg = Int32()
        msg.data = self.counter_

        # Publish message
        if( self.counter_ < 1000 ):
            self.pub_int_.publish(msg)

        # Increment counter
        self.counter_ += 1

        return

    
    ###########################
    # Publish string messsage
    ###########################
    def publish_str_message(self):

        # Build string
        str = 'Counter = %d'% self.counter_

        # Define message
        msg = String()
        msg.data = str

        # Publish message
        self.pub_str_.publish(msg)

        return


    ###########################
    # Publish image messsage
    ###########################
    def publish_img_message(self):

        rows = 480;
        cols = 640;
        radius = 50;

        img = np.zeros((rows,cols,3), np.uint8)

        # Set the circle movement freqency
        freq = 0.1; # Hz
  
        # Get current time
        t = rospy.Time.now().to_sec()

        # Compute the center
        xc = int( cols/2 + int (np.sin(2.0*np.pi*freq*t)*cols/4) )
        yc = int( rows/2 + int (np.sin(2.0*np.pi*freq/1.5*t)*rows/4) )

        # Draw a circles at the current location
        cv.circle(img, (xc,yc), radius, (0,0,255), -1)
        cv.circle(img, (xc,yc+radius), radius-25, (0,0,255), -1)
        cv.circle(img, (xc,yc-radius), radius-25, (0,0,255), -1)

        # Publish message
        self.pub_img_.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        
        # Display image
        self.display_image('Source', img, True )

        return


    ####################
    # Display an image
    ####################
    def display_image(self, title_str, img, disp_flag ):

        if( disp_flag ):
            # Display the given image
            cv.namedWindow(title_str, cv.WINDOW_NORMAL)
            cv.imshow(title_str, img)
            cv.waitKey(3)

            # Add window to active window list
            if not ( title_str in ACTIVE_WINDOWS ):
                ACTIVE_WINDOWS.append(title_str)
        else:
            if( title_str in ACTIVE_WINDOWS):
                cv.destroyWindow(title_str)
                ACTIVE_WINDOWS.remove(title_str)
        return



#################    
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('publisher_py_node')
    print("Publisher py node initialized")
    
    # Start tester
    try:
        PublishNode()
    except rospy.ROSInterruptException:
        pass



    
