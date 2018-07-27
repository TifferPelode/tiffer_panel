#!/usr/bin/env python
#coding=utf8
import socket

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(('47.52.210.190', 8766))

    try:
        global move_base

        rospy.init_node('weChat2ROS', anonymous=True)
        rate = rospy.Rate(1) # 10hz

        marker = list()
        find_words = list()
        find_words.append(u'原点')

        marker.append(Pose(Point(0.499, 0.975, 0.0), \
            Quaternion(0.0, 0.0, 0.008, 1.000)))

        move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move_base server")
        rospy.loginfo("Start nav test")
        while True:
            buffer = s.recv(1024)
            #if (len(buffer) <= 0):
            #    break
            if (buffer == b'2'):
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = marker[0]
                move(goal)
            #print buffer

    except rospy.ROSInterruptException:
        rospy.loginfo("InterruptException.")

def move(goal):
    global move_base
    move_base.send_goal(goal)
    timeout = move_base.wait_for_result(rospy.Duration(200))

    if not timeout:
        move_base.cancel_goal()
        rospy.loginfo("time out to achieving goal")
    else:
        state = move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("get the goal")

if __name__ == '__main__':
    main()
