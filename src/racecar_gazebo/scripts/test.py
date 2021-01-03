#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def subscriber_cb(data):
    pub.publish(data.position[4]) # left_front

def main():
    global pub

    sub = rospy.Subscriber('/racecar/joint_states', JointState, subscriber_cb, queue_size=1)
    pub = rospy.Publisher('/test', Float64, queue_size=1)

if __name__ == '__main__':
    
    rospy.init_node("test")
    main()
    rospy.spin()
