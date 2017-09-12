#!/usr/bin/env python
import baxter_interface
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node("baxter_camera")
display_pub= rospy.Publisher('/robot/xdisplay',Image,queue_size=1)

def republish(msg):
        """
            Sends the camera image to baxter's display
        """             
	bridge = CvBridge()
	cvimage = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
	resized_image = cv2.resize(cvimage, (1024, 600)) 
	msg_trans = bridge.cv2_to_imgmsg(resized_image, encoding="passthrough")
        display_pub.publish(msg_trans)
	#rospy.sleep(1)

sub = rospy.Subscriber('/camera/rgb/image_rect_color', Image,republish,None,1)
rospy.spin()

