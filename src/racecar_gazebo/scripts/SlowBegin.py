#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

# This script is in order to begin slowly to avoid odom mistakes

def GoalReceiveCallback(msg):
	rospy.loginfo("SlowBeginning... vel=%d" %(vel.linear.x))
	for i in range(0,10000): 	# publish for a piece of time
		pub.publish(vel)
	rospy.loginfo("EndBeginning...")

if __name__ == '__main__':
	# values
	vel = Twist()
	vel.linear.x = 0.4 # velocity x
	vel.angular.z = 0 # default
	# init ROS
	rospy.init_node('SB', anonymous=False)
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	rospy.loginfo("SB_Runing")
	rospy.Subscriber("/move_base_simple/goal", PoseStamped, GoalReceiveCallback)
	rospy.spin()
	
