#!/usr/bin/env python

"""
imports
"""
import rospy
import numpy as np
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped, PointStamped
from tf.transformations import quaternion_from_euler

begin_flag = Bool()
reach_flag = Bool()
receive_flag = Bool()
begin_flag.data = False
reach_flag.data = True # default True to begin the 1st pub
receive_flag.data = False
Goals = []
i = 1


def begin_sub_cb(data):
    """
    This is the callback of begin_flag_subscriber.
    param: data: PoseStamped
    return: None
    """
    global i
    # get flag
    begin_flag.data = True

    # info
    rospy.loginfo("Begin flag received!")

    i = 1
    Goals.append(data)


def reach_sub_cb(data):
    """
    This is the callback of reach_flag_subsciber.
    param: data: MoveBaseActionResult
    return: None
    """
    global i
    # reached previous goal 
    reach_flag.data = True

    # info
    rospy.loginfo("%d Goal reached!..." %i)
    
    i = i + 1


def goal_sub_cb(data):
    """
    This is the callback of goal_set_subsciber.
    param: data: PointStamped
    return: None
    """
    global i
    # get Goals in order
    goal = PoseStamped()
    
    z = 0.8
    q = quaternion_from_euler(0, 0, z)

    goal.header = data.header
    goal.pose.position = data.point
    goal.pose.orientation.x = q[0]
    goal.pose.orientation.y = q[1]
    goal.pose.orientation.z = q[2]
    goal.pose.orientation.w = q[3]

    Goals.append(goal)
    # receive one of the goals
    receive_flag.data = True
    # info
    rospy.loginfo("%d Goal set successfully!..." %i)
    i = i + 1


def publisher_cb(_):
    """
    This is the callback of a timer with 20Hz, checking if it is time to publish goals
    param: None
    return: None
    """
    # all condition
    if begin_flag.data and reach_flag.data and receive_flag.data:
        if Goals == []:
            rospy.loginfo('Final Goal Reach!...')
        else:
            goal_pub.publish(Goals.pop(0))
        reach_flag.data = False


def distance(x1, y1, x2, y2):
    """
    calculate the distance between two points
    param: x1, y1, x2, y2
    return: dis
    """
    dis = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    return dis


def rank():
    pass

    
def main():
    """
    main function start
    """
    # global
    global goal_pub

    # init node
    rospy.init_node("goal_sub_pub_cmd")
    
    # Subscribers
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, reach_sub_cb, queue_size=1)
    rospy.Subscriber('/move_base_simple/my_goal', PoseStamped, begin_sub_cb, queue_size=1)
    rospy.Subscriber('/clicked_point', PointStamped, goal_sub_cb, queue_size=1)
    
    # Publisher
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    
    # timer
    rospy.Timer(rospy.Duration(0.5), publisher_cb)

    # spin
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
