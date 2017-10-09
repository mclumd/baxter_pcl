#!/usr/bin/env python
from sensor_msgs.msg import Range 
import rospy

rospy.init_node("baxter_ir")

def getrange(msg):
        """
            Sends the camera image to baxter's display
        """             
	print round(msg.range,2)

sub = rospy.Subscriber('/robot/range/left_hand_range/state', Range,getrange,queue_size = 1)
rospy.spin()


