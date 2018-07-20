#!/usr/bin/env python
#coding=utf8

# license removed for brevity
import requests
import itchat

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal



@itchat.msg_register(itchat.content.TEXT)
def wcMsg(msg):
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

        pub = rospy.Publisher('weChat_cmd', String, queue_size=10)

        if msg['Text'] == u'前进':
            cmd_str = 'Up'
            itchat.send(u'ROS机器人-前进中', 'filehelper')

        elif msg['Text'] == u'后退':
            cmd_str = 'Down'
            itchat.send(u'ROS机器人-后退中', 'filehelper')

        elif msg['Text'] == u'右转':
            cmd_str = 'Right'
            itchat.send(u'ROS机器人-右转中', 'filehelper')

        elif msg['Text'] == u'左转':
            cmd_str = 'Left'
            itchat.send(u'ROS机器人-左转中', 'filehelper')

        elif msg['Text'] == u'停':
            cmd_str = 'Stop'
            itchat.send(u'ROS机器人-不走了', 'filehelper')

        elif find_words[0] in msg['Text']:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = marker[0]
            move(goal)
	    print type(msg['Text'])

        pub.publish(cmd_str)
        rate.sleep()
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

    itchat.auto_login(True)
    itchat.run()
