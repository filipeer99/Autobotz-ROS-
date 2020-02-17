#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose

def PoseCallback(pose_message):
    print('X -> ', pose_message.x)
    print('Y -> ', pose_message.y)

rospy.init_node('position')
rospy.Subscriber('turtle1/pose', Pose, PoseCallback)
rospy.spin()