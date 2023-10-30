#!/usr/bin/env python3
import sys
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
from std_msgs.msg import Int16,Int16MultiArray



def callback(imageIn):
    frame = bridge.imgmsg_to_cv2(imageIn,"bgr8")
    
    rpub.publish(distance_from_center)
    tpub.publish(angle)

if __name__=='__main__':
    bridge = CvBridge()
    rospy.init_node("vision" , anonymous=True)
    rpub = rospy.Publisher("center",Int16,queue_size=10)
    tpub = rospy.Publisher("turn",Int16,queue_size=10)
    rospy.Subscriber("images", Image, callback)
    #loop
    rospy.spin()
    #listner()
    cv2.destroyAllWindows()
