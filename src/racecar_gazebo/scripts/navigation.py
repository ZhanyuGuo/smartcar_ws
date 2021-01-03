#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

# def publisher_cb(_):
#     pub.publish(ackermann_control)

def subscriber_cb(data):
    ackermann_control.drive.speed = data.linear.x
    ackermann_control.drive.steering_angle = data.angular.z
    pub.publish(ackermann_control)

def main():
    global pub
    global ackermann_control

    ackermann_control = AckermannDriveStamped()
    rospy.Subscriber('/cmd_vel', Twist, subscriber_cb, queue_size=1)
    pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=1)
#     rospy.Timer(rospy.Duration(0.01), publisher_cb)

if __name__ == '__main__':
    
    rospy.init_node("my_navigation")
    main()
    rospy.spin()
