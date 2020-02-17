#!/usr/bin/env python

import rospy
import math
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

x = 0
y = 0
theta = 0


def PoseCallback(pose_message):
    global x
    global y
    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta


def move(speed, distance, is_foward):
    velocity_message = Twist()
    global x, y
    x0 = x
    y0 = y

    if (is_foward):
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(10)
    cmd_vel_topic = '/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    while True:
        rospy.loginfo("Turtlesim moves fowards")
        velocity_publisher.publish(velocity_message)

        loop_rate.sleep()

        distance_moved = distance_moved + abs(0.5 * math.sqrt(((x - x0) ** 2) + ((y - y0) ** 2)))
        print(distance_moved)

        if not (distance_moved < distance):
            rospy.loginfo("reached")
            break

    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)


def rotate(angular_speed_degree, relative_angle_degree, clockwise):
    global theta

    velocity_message = Twist()
    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.y = 0

    theta0 = theta
    angular_speed = math.radians(abs(angular_speed_degree))

    if (clockwise):
        velocity_message.angular.z = -abs(angular_speed)
    else:
        velocity_message.angular.z = abs(angular_speed)

    angle_moved = 0.0
    loop_rate = rospy.Rate(10)
    cmd_vel_topic = '/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()

    while (True):
        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(velocity_message)
        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1 - t0) * angular_speed_degree
        loop_rate.sleep()

        if (current_angle_degree > relative_angle_degree):
            rospy.loginfo("reached")
            break

    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)


if __name__ == '__main__':
    try:
        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        cmd_vel_topic = '/turlte1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, PoseCallback)
        time.sleep(2)

        move(1.0, 2.0, True)
        rotate(30, 90, False)
        move(1.0, 2.0, True)
        rotate(30, 90, False)
        move(1.0, 2.0, True)
        rotate(30, 90, False)
        move(1.0, 2.0, True)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")
