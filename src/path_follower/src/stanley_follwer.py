#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
import pickle
from stanley import StanleyControl

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Float64

class StanleyController(object):
    def __init__(self):
        self.rear_x = 0.0
        self.rear_y = 0.0
        self.yaw = 0.0

        with open("/home/hayoung/personal/grepp_project_ws/src/map/path.pkl", "rb") as f:
            self.path = pickle.load(f)

        self.ego_pose_sub = rospy.Subscriber("ego_pose", PoseStamped, self.PoseCallBack)

    def PoseCallBack(self, msg):
        self.rear_x = msg.pose.position.x
        self.rear_y = msg.pose.position.y
        orientation_q = msg.pose.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

    def GetSteeringAngleMsg(self):
        delta = StanleyControl(self.rear_x, self.rear_y, self.yaw,
                               self.path['x'], self.path['y'], self.path['yaw'],
                               L=3.14)
        steering_angle_msg = Float64()
        steering_angle_msg.data = delta
        return steering_angle_msg


class PIDController(object):
    def __init__(self):
        self.current_speed = 0.0
        self.p_gain = 0.5
        self.speed_sub = rospy.Subscriber("ego_speed", Float64, self.CurSpeedCallBack)

    def GetAccCmdMsg(self, target_speed):
        acc_cmd = -self.p_gain * (self.current_speed - target_speed)

        acc_cmd_msg = Float64()
        acc_cmd_msg.data = acc_cmd

        return acc_cmd_msg

    def CurSpeedCallBack(self, msg):
        self.current_speed = msg.data

if __name__ == '__main__':
    rospy.init_node("stanley_follower_node")

    steering_angle_pub = rospy.Publisher("steering_angle_cmd", Float64, queue_size=1)
    acc_cmd_pub = rospy.Publisher("acc_cmd", Float64, queue_size=1)

    stanley = StanleyController()
    controller = PIDController()

    target_speed = 10.0

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        steering_angle_msg = stanley.GetSteeringAngleMsg()
        steering_angle_pub.publish(steering_angle_msg)

        acc_cmd_msg = controller.GetAccCmdMsg(target_speed)
        acc_cmd_pub.publish(acc_cmd_msg)
        r.sleep()
