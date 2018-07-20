#!/usr/bin/env python
#coding=utf-8

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi

class OutAndBack():
    def __init__(self):
        # 给出节点的名字
        rospy.init_node('out_and_back', anonymous=False)

        # 设置rospy在程序退出时执行的关机函数      
        rospy.on_shutdown(self.shutdown)

        # Publisher（发布器）：发布机器人运动的速度
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # 我们将用多快的速度更新控制机器人运动的命令?
        rate = 20

        # 设定相同的值给rospy.Rate()
        r = rospy.Rate(rate)

        # 设定前进的线速度为0.2m/s
        linear_speed = 0.2

        # 设定行程距离为1.0m
        goal_distance = 1.0

        # 设定转动速度1.0rad/s .即大约6秒转一圈
        angular_speed = 1.0

        # Set the angular tolerance in degrees converted to radians
        angular_tolerance = radians(2.5)

        # 设定转动弧度： Pi 弧度 (180 度)
        goal_angle = pi

        # 初始化这个tf监听器
        self.tf_listener = tf.TransformListener()

        # 给tf一些时间让它填补它的缓冲区
        rospy.sleep(2)

        # 设置odom坐标系
        self.odom_frame = '/odom'

        # 询问机器人使用的是/base_link坐标系还是/base_footprint坐标系
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  

        # 初始化了一个Point类型的变量
        position = Point()

        # 往返2次行程
        for i in range(2):
            # 初始化运动命令
            move_cmd = Twist()

            # 设定前进速度
            move_cmd.linear.x = linear_speed

            # 得到开始的姿态信息（位置和转角）    
            (position, rotation) = self.get_odom()

            x_start = position.x
            y_start = position.y

            # 随时掌控机器人行驶的距离
            distance = 0

            # 进入循环，沿着一边移动
            while distance < goal_distance and not rospy.is_shutdown():
                # 发布一次Twist消息 和 sleep 1秒        
                self.cmd_vel.publish(move_cmd)

                r.sleep()

                # 给出正确的姿态信息（位置和转角）
                (position, rotation) = self.get_odom()

                # 计算相对于开始位置的欧几里得距离（即位移）
                distance = sqrt(pow((position.x - x_start), 2) + 
                                pow((position.y - y_start), 2))

            # 在转动机器人前，停止它
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)

            # 给旋转配置运动命令
            move_cmd.angular.z = angular_speed

            # 跟踪记录最后的角度
            last_angle = rotation

            # 跟踪我们已经转动了多少角度
            turn_angle = 0

            while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
                # 发布一次Twist消息 和 sleep 1秒  
                self.cmd_vel.publish(move_cmd)
                r.sleep()

                # 给出正确的姿态信息（位置和转角）
                (position, rotation) = self.get_odom()

                # 计算自每次的旋转量
                delta_angle = normalize_angle(rotation - last_angle)

                # 添加到正在运行的合计里
                turn_angle += delta_angle
                last_angle = rotation

            # 下一航段之前停止机器人
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)

        # 为了机器人好，停止它
        self.cmd_vel.publish(Twist())

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def shutdown(self):
        # 当关闭这个节点时，总是让机器人停止不动。
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        OutAndBack()
    except:
        rospy.loginfo("Out-and-Back node terminated.")
