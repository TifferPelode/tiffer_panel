#!/usr/bin/env python
import rospy 
from nav_msgs.msg import Path
from math import sqrt

def callback(data):
    len_ = len(data.poses)
    print "total points: %d" % len_

    sum_ = 0.0

    for i in range(len_ - 1):
        sum_ += sqrt(pow((data.poses[i+1].pose.position.x - data.poses[i].pose.position.x),2) + pow((data.poses[i+1].pose.position.y - data.poses[i].pose.position.y), 2))
    
    print sum_


def listener():
    rospy.init_node('cal_path', anonymous=False)
    rospy.Subscriber("/move_base/NavfnROS/plan", Path, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
