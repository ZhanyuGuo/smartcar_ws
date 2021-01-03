#!/usr/bin/env python
# -*- coding: utf-8 -*- 

#ZJ2020

'''
用法：这个文件在launch文件中作为一个节点启动后，会订阅两个话题：
1.zj_control_cmd/speed std_msgs/Float64  速度期望 （目前单位是轮子的角速度（rad/s））
2.zj_control_cmd/turn  std_msgs/Float64 舵机转角 输入角度 -45 - +45 度(deg)  +左转，-右转
'''

import rospy
import math
import tf2_ros
import tf
from tf2_ros import transform_listener
import control_msgs
from control_msgs.msg import JointControllerState
import std_msgs
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped


#publisher(s)
LRW_topic   = '/racecar/left_rear_wheel_velocity_controller/command'
RRW_topic   = '/racecar/right_rear_wheel_velocity_controller/command'
LFW_topic   = '/racecar/left_front_wheel_velocity_controller/command'
RFW_topic   = '/racecar/right_front_wheel_velocity_controller/command'
LSH_topic   = '/racecar/left_steering_hinge_position_controller/command'
RSH_topic   = '/racecar/right_steering_hinge_position_controller/command'

pub_vel_LRW = rospy.Publisher(LRW_topic, Float64, queue_size = 1)#左后速度
pub_vel_RRW = rospy.Publisher(RRW_topic, Float64, queue_size = 1)
pub_vel_LFW = rospy.Publisher(LFW_topic, Float64, queue_size = 1)
pub_vel_RFW = rospy.Publisher(RFW_topic, Float64, queue_size = 1)
pub_pos_LSH = rospy.Publisher(LSH_topic, Float64, queue_size = 1)#左轮舵机
pub_pos_RSH = rospy.Publisher(RSH_topic, Float64, queue_size = 1)

#期望速度变量
speed_want=Float64()
turn_want=Float64()
speed_want.data=0
turn_want.data=0

#某个控制器的回调函数，用来同步发送频率（由gazebo发来，收到一次，就发送一次期望速度给gazebo)
#目前是轮子角速度
def my_control_callback(data):
    pub_vel_LRW.publish(speed_want)
    pub_vel_RRW.publish(speed_want)
    pub_vel_LFW.publish(speed_want)
    pub_vel_RFW.publish(speed_want)
    pub_pos_LSH.publish(turn_want)
    pub_pos_RSH.publish(turn_want)

#速度设置信息话题的回调函数
def set_throttle_steer(data):
    speed_want.data=data.drive.speed*40
    turn_want.data=data.drive.steering_angle

if __name__ == '__main__':

    try:
        rospy.init_node('my_controller_cmd', anonymous = True)#初始化节点
        rospy.Subscriber("/racecar/left_front_wheel_velocity_controller/state", 
                         JointControllerState, 
                         my_control_callback)#订阅任意一个控制的state，用回调函数作为控制指令的同步
                         #(回调函数被调用一次，就发布一次控制指令)
        rospy.Subscriber("/racecar/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

