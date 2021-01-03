#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, String

def start():
    rospy.init_node("start_cmd")
    
    flag = Bool()
    flag.data = True
    pub = rospy.Publisher('/begin_flag', Bool, queue_size=1)
    
    pub.publish(flag)
    rospy.loginfo('start')
    
if __name__ == "__main__":
    try:
        start()
    except rospy.ROSInterruptException:
        pass