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

# SubscribeNode class definition
class SubscribeNode():
    def __init__(self):
        """Template package subscriber node class"""

        # Variables
        self.thres_ = 100
        
        # Dynamic reconfigure variables
        self.dyn_config = []
        self.dyn_reconfig_double = 0.0

        # ROS Parameters
  
        # ROS dynamic reconfigure server
        self.srv = Server(PublishNodeDynCfgConfig, self.dyn_reconfig_callback)
        
        # ROS Time

        # ROS Topic Publisher

        # ROS Topic Subscribers
        self.sub_int_ = rospy.Subscriber('int_msg', Int32,
                                         self.int_message_callback,
                                         queue_size=10)
        
        self.sub_str_ = rospy.Subscriber('str_msg', String,
                                         self.str_message_callback,
                                         queue_size=10)
        
        self.sub_img_ = rospy.Subscriber('img_msg', Image,
                                         self.img_message_callback,
                                         queue_size=10)

        # Define ROS rate

        # Set up CV Bridge
        self.bridge = CvBridge()
        
        # Report out node construction
        rospy.loginfo('subscribe_node running!')

        # Enter ROS loop
        rospy.spin()
        
        return

    
    ################################
    # Dynamic Reconfigure callback
    ################################
    def dyn_reconfig_callback(self, config, level):
        self.thres_ = config['int_param']
        self.dyn_reconfig_double = config['double_param']
        return config

    ######################################################################
    # int_message_callback: Function to display integer message received
    ######################################################################
    def int_message_callback(self, msg):
        rospy.loginfo('Recieved int = %d', msg.data)
        return
    
    #####################################################################
    # str_message_callback: Function to display string message received
    #####################################################################
    def str_message_callback(self, msg):
        rospy.loginfo('Recieved str = %s', msg.data)
        return
    
    #########################
    # Camera image callback
    #########################
    def img_message_callback(self, rgb_msg):

        # Get the camera image and make a copy
        img = CvBridge().imgmsg_to_cv2(rgb_msg, "bgr8" )

        src_gray = cv.cvtColor( img, cv.COLOR_BGR2GRAY )
        src_gray = cv.blur( src_gray, (3,3) )

        canny_output = cv.Canny(src_gray, self.thres_, self.thres_ * 2)
    
        contours, _ = cv.findContours(canny_output, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        
        contours_poly = [None]*len(contours)
        boundRect = [None]*len(contours)
        centers = [None]*len(contours)
        radius = [None]*len(contours)
        for i, c in enumerate(contours):
            contours_poly[i] = cv.approxPolyDP(c, 3, True)
            boundRect[i] = cv.boundingRect(contours_poly[i])
            centers[i], radius[i] = cv.minEnclosingCircle(contours_poly[i])
    
    
        drawing = np.zeros((canny_output.shape[0], canny_output.shape[1], 3),
                           dtype=np.uint8)
    
        for i in range(len(contours)):
            color = (0, 255,255)
            cv.drawContours(drawing, contours_poly, i, color)
            cv.rectangle(drawing, (int(boundRect[i][0]),
                                   int(boundRect[i][1])),
                         (int(boundRect[i][0]+boundRect[i][2]),
                          int(boundRect[i][1]+boundRect[i][3])),
                         color, 2)
            
            cv.circle(drawing, (int(centers[i][0]),
                                int(centers[i][1])), int(radius[i]), color, 2)

        self.display_image('Target Object', drawing, True)
        
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
    rospy.init_node('subscriber_py_node')
    print("subscriber python node initialized")
    
    # Start tester
    try:
        SubscribeNode()
    except rospy.ROSInterruptException:
        pass



    
