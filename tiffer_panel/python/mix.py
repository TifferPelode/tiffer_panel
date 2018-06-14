#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import sys

reload(sys)
sys.setdefaultencoding('utf-8')


def cb(data):
    if find_words[0] in data.data:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = marker[0]
        move(goal)
    elif find_words[1] in data.data:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = marker[1]
        move(goal)


def move(goal):
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
    try:
        marker = list()
        find_words = list()
        find_words.append(u'饮水机')
        find_words.append(u'原点')
        rospy.init_node("STC",anonymous=True)
        rospy.Subscriber("xfspeech", String, cb)

        marker.append(Pose(Point(3.284, 5.337, 0.0), \
            Quaternion(0.0, 0.0, -0.245, 0.970)))
        marker.append(Pose(Point(3.563, 10.989, 0.0), \
            Quaternion(0.0, 0.0, 1.000, 0.029)))

        cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move_base server")
        rospy.loginfo("Start nav test")

        rospy.spin()
    except rospy.ROSInterruptException:
rospy.loginfo("InterruptException.")
