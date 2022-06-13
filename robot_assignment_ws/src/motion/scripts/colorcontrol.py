#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np

rospy.init_node('opencv_example', anonymous=True)
bridge = CvBridge()



def image_callback(img_msg):
    pub=rospy.Publisher('/witsdetector',String,queue_size=10)
    try:
    	# read in the image from ros to convert to a opencv image
        cv_image = bridge.imgmsg_to_cv2(img_msg,"bgr8")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    
 
    # convert to hsv
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) 
    # get the green of the utility bot
    green=np.uint8([[[5,5,255]]])
    hsvGreen=cv2.cvtColor(green, cv2.COLOR_BGR2HSV) 
    # determine its upper and lower bounds for detecting the utility bot
    ming=hsvGreen[0][0][0]-1,100,100
    maxg=hsvGreen[0][0][0]+1,255,255
    # create a mask to isolate the green in that range
    mask_g = cv2.inRange(hsv, ming,maxg) 
   
    # if the mask does not contain isolations of that colour i.e. all numbers in mask are 0 (black):
    if (np.all((mask_g==0))):
    	# no utility cart in view
    	pub.publish("No")
    else:
        # if the cart contains isolations of that colour
    	pub.publish("Yes")
    	
   
   #sub_once.unregister()
    
   
    
   
global sub_once
sub_once = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)


while not rospy.is_shutdown():
     
      rospy.spin()
      break

