#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
from aip import AipSpeech
import rospy
import rospkg
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

reload(sys)
sys.setdefaultencoding('utf-8')

APP_ID = '9613889'
API_KEY = 'XEjkM5h9OXYN0SN0EmCCc0Hu'
SECRET_KEY = 'bcc92f1d276d8cc10f322c5a5f64ca0e'

client = AipSpeech(APP_ID, API_KEY, SECRET_KEY)

def save_result(res):
    f = open('/home/tiffer/tiffer-catkin/src/tiffer_panel/file/result', 'w')
    f.write(res)

def get_file_content(filePath):
    with open(filePath, 'rb') as fp:
        return fp.read()

'''def move(goal):
    move_base.send_goal(goal)
    timeout = move_base.wait_for_result(rospy.Duration(200))

    if not timeout:
        move_base.cancel_goal()
        rospy.loginfo("time out to achieving goal")
    else:
        state = move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("get the goal")'''

if __name__ == "__main__":

    result = client.asr(get_file_content('/home/tiffer/tiffer-catkin/src/tiffer_panel/file/16k.wav'), 'wav', 16000, {
        'lan': 'zh',
    })
    print(result['result'][0])
    r = result['result'][0]
    save_result(r)

    try:
        marker = list()
        find_words = list()
        find_words.append(u'饮水机')
        find_words.append(u'原点')
        rospy.init_node("Tiffer",anonymous=True)
        #rospy.Subscriber("tiffer", String, cb)

        marker.append(Pose(Point(3.284, 5.337, 0.0), \
            Quaternion(0.0, 0.0, -0.245, 0.970)))
        marker.append(Pose(Point(3.563, 10.989, 0.0), \
            Quaternion(0.0, 0.0, 1.000, 0.029)))

        #cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        #move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        #move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move_base server")
        rospy.loginfo("Start nav test")

        for i in range(len(find_words)):
            if find_words[i] in r:
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = marker[i]
                #move(goal)

    except rospy.ROSInterruptException:
        rospy.loginfo("InterruptException.")
