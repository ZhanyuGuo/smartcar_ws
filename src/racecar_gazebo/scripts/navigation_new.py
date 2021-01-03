#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import time
import math
import numpy as np
import numpy.linalg as LA
from move_base_msgs.msg import MoveBaseActionGoal

#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
'''-------------global param----------------'''
point_width = 40
mid_point = 100
'''-------------global param----------------'''
pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=1)
pub_curvature = rospy.Publisher("my_control_cmd/curvature", Float64, queue_size = 1)


pub_point1 = rospy.Publisher("point1", PointStamped, queue_size = 1)
pub_point2 = rospy.Publisher("point2", PointStamped, queue_size = 1)
pub_point3 = rospy.Publisher("point3", PointStamped, queue_size = 1)

point1 = PointStamped()
point2 = PointStamped()
point3 = PointStamped()

ack = AckermannDriveStamped()

min_curvature = Float64(1000.0)
'''-----------------stop car------------------'''
goal_point_x = Float64()
goal_point_y = Float64()
odom_point_x = Float64()
odom_point_y = Float64()
odom_yaw = Float64()
def odom_callback(data):
    global odom_point_x
    global odom_point_y
    global odom_yaw
    odom_point_x = data.pose.pose.position.x
    odom_point_y = data.pose.pose.position.y
    odom_yaw = data.twist.twist.angular.z
def get_goal_callback(data):
    global goal_point_x
    global goal_point_y
    goal_point_x = data.goal.target_pose.pose.position.x
    goal_point_y = data.goal.target_pose.pose.position.y
'''-----------------sopt car------------------'''

def lenth_cal(x1,x2,y1,y2):
    x = (x1 - x2)**2
    y = (y1 - y2)**2
    l = np.sqrt(x+y)
    return l

def radius_cal(a,b,c):
    p = (a+b+c)/2.0
    s = np.sqrt(p*(p-a)*(p-b)*(p-c))
    r = (a*b*c)/4.0/s
    return r



def cmd_vel_callback(data):
    global min_curvature
    global ack

    stop_flag = Float64(lenth_cal(float(odom_point_x) , float(goal_point_x) , float(odom_point_y) , float(goal_point_y)))
    if stop_flag.data <= 1.5:
        ack.drive.speed = 0
        ack.drive.steering_angle = 0
    else:
        ack.drive.steering_angle=data.angular.z*1.0
        if min_curvature.data <= 3:
            ack.drive.speed= data.linear.x*0.8  #20
        else:
            ack.drive.speed= data.linear.x*1.0   #38
        
    pub_curvature.publish(min_curvature)
    pub.publish(ack)




def path_callback(data):       
    if len(data.poses) <= point_width*2:
        index1 = 0
        index2 = (len(data.poses)-1)//2
        index3 = len(data.poses)-1
    elif (len(data.poses) > (point_width*2)) and (len(data.poses) <= (mid_point+point_width)):
        index1 = (len(data.poses)-1)-(point_width*2)
        index2 = (len(data.poses)-1)-point_width
        index3 = len(data.poses)-1
    elif len(data.poses) > (mid_point+point_width):
        index1 = mid_point-point_width
        index2 = mid_point
        index3 = mid_point+point_width
    else:
        index1 = index2 = index3 = 0


    point1.header.frame_id = 'map'
    point1.point.x = data.poses[index1].pose.position.x
    point1.point.y = data.poses[index1].pose.position.y
    pub_point1.publish(point1)

    point2.header.frame_id = 'map'
    point2.point.x = data.poses[index2].pose.position.x
    point2.point.y = data.poses[index2].pose.position.y
    pub_point2.publish(point2)

    point3.header.frame_id = 'map'
    point3.point.x = data.poses[index3].pose.position.x
    point3.point.y = data.poses[index3].pose.position.y
    pub_point3.publish(point3)

    if index1 >= 0:
        global min_curvature
        min_curvature = Float64(1000.0)
        for i in range(0,index1): 
            len1 = lenth_cal(data.poses[index1-i].pose.position.x , data.poses[index2-i].pose.position.x , data.poses[index1-i].pose.position.y , data.poses[index2-i].pose.position.y)
            len2 = lenth_cal(data.poses[index2-i].pose.position.x , data.poses[index3-i].pose.position.x , data.poses[index2-i].pose.position.y , data.poses[index3-i].pose.position.y)
            len3 = lenth_cal(data.poses[index1-i].pose.position.x , data.poses[index3-i].pose.position.x , data.poses[index1-i].pose.position.y , data.poses[index3-i].pose.position.y)
            curvature = Float64(radius_cal(len1 , len2 , len3))
            if curvature.data <= min_curvature.data:
                min_curvature.data = curvature.data
    #k1 = Float64((data.poses[index1].pose.position.y - data.poses[0].pose.position.y)/(data.poses[index1].pose.position.x - data.poses[0].pose.position.x))
    #k2 = Float64((data.poses[index2].pose.position.y - data.poses[index1].pose.position.y)/(data.poses[index2].pose.position.x - data.poses[index1].pose.position.x))
    #k_err = Float64(k2.data - k1.data)
   
    
if __name__ == '__main__':
    try:
        rospy.init_node('my_navigation_cmd', anonymous = True)#初始化节点
        rospy.Subscriber("cmd_vel", 
                         Twist, 
                         cmd_vel_callback)
        rospy.Subscriber("/move_base/TebLocalPlannerROS/global_plan", 
                         Path, 
                         path_callback)
        rospy.Subscriber("/move_base/goal", 
                         MoveBaseActionGoal, 
                         get_goal_callback)
        rospy.Subscriber("/odom", 
                         Odometry, 
                         odom_callback) 
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
