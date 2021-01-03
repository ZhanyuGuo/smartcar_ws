#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, String

def cb(data):
    rospy.loginfo('received!')

def receive():
    rospy.init_node("receiver")
    rospy.Subscriber('/begin_flag', Bool, cb, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    try:
        receive()
    except rospy.ROSInterruptException:
        pass