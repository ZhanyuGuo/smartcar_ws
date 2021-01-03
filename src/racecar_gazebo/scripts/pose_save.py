#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped

i = 1

def savepose_cb(data):
    global i
    
    writedate_x(data.point.x)
    writedate_y(data.point.y)
    rospy.loginfo("%d set successfully!" % i)
    i += 1
    if i > 11:
        rospy.signal_shutdown("end of setting points.")


def writedate_x(data):  
   with open('/home/gzy/pose.txt','a') as file_handle:
       file_handle.write("{}\n".format(data))

def writedate_y(data):    
   with open('/home/gzy/pose.txt','a') as file_handle:
       file_handle.write("{}\n".format(data))


def main():
    #init node
    rospy.init_node("save_pose_node")

    #Subscriber
    rospy.Subscriber('/clicked_point', PointStamped, savepose_cb, queue_size=1)

    rospy.loginfo("pose begin saving...")

    # spin
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
