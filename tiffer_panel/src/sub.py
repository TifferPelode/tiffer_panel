#!/usr/bin/env python
import sys
import os

import rospy 
from move_base import *

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "TP : %s", data.data)

def listener():
    rospy.init_node('sub_path', anonymous=True)
    rospy.Subscriber("/move_base/NavfnROS/plan", "nav_msgs/GetPlan", callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
