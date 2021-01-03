#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Int8, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

pos = []
with open('/home/gzy/pose.txt', 'r') as file_handle:
    for i in range(11):
        x = float(file_handle.readline().replace('\n', ''))
        y = float(file_handle.readline().replace('\n', ''))
        pos.append([x, y])


def get_degree(rad):
    """
    This function transfers radians to degrees.
    param: rad
    return: degree
    """
    degree = math.degrees(rad)
    return degree

def get_cur(x, y):
    """
    This function gets current point(starting_point)'s num
    param: x
           y
    return: int
    """
    for i in [1, 2, 3, 4, 6, 7, 8, 9, 10]:
        if pos[i - 1][0] - 1 < x < pos[i - 1][0] + 1 and pos[i - 1][1] - 0.75 < y < pos[i - 1][1] + 0.75:
            return i

    if pos[5 - 1][0] - 1 < x < pos[5 - 1][0] + 1 and pos[5 - 1][1] - 1.25 < y < pos[5 - 1][1] + 1.25:
        return 5
    
    if pos[11 - 1][0] - 1.5 < x < pos[11 - 1][0] + 1.5 and pos[11 - 1][1] - 0.75 < y < pos[11 - 1][1] + 0.75:
        return 11
    return -1


def get_nxt(cur, angle):
    """
    This function gets next point's num
    param: cur: int
           angle: float
    return: int
    """
    if cur == 1:
        if -45 < angle < 45:
            nxt = 2
        elif -135 < angle < -45:
            nxt = 9
        else:
            nxt = -1
    elif cur == 2:
        if angle > 135 or angle < -135:
            nxt = 1
        elif -135 < angle < -45:
            nxt = 10
        elif -45 < angle < 45:
            nxt = 4
        else:
            nxt = -1
    elif cur == 3:
        if angle > 0:
            nxt = 4
        else:
            nxt = 11
    elif cur == 4:
        if angle > 135 or angle < -135:
            nxt = 3
        elif -135 < angle < -45:
            nxt = 5
        else:
            nxt = -1
    elif cur == 5:
        if 45 < angle < 135:
            nxt = 4
        elif angle > 135 or angle < -135:
            nxt = 11
        elif -135 < angle < -45:
            nxt = 6
        else:
            nxt = -1
    elif cur == 6:
        if 45 < angle < 135:
            nxt = 5
        elif angle > 135 or angle < -135:
            nxt = 7
        else:
            nxt = -1
    elif cur == 7:
        if 45 < angle < 135:
            nxt = 10
        elif -45 < angle < 45:
            nxt = 6
        elif angle > 135 or angle < -135:
            nxt = 8
        else:
            nxt = -1
    elif cur == 8:
        if 45 < angle < 135:
            nxt = 10
        elif -45 < angle < 45:
            nxt = 7
        elif angle > 135 or angle < -135:
            nxt = 9
        else:
            nxt = -1
    elif cur == 9:
        if -45 < angle < 45:
            nxt = 8
        elif 45 < angle < 135:
            nxt = 1
        else:
            nxt = -1
    elif cur == 10:
        if 0 < angle < 90:
            nxt = 11
        elif 90 < angle < 180:
            nxt = 2
        elif -90 < angle < 0:
            nxt = 7
        else:
            nxt = 8
    elif cur == 11:
        if -170 < angle < -90:
            nxt = 10
        elif -90 < angle < 30:
            nxt = 5
        elif 30 < angle < 90:
            nxt = 3
        else:
            nxt = -1
    else:
        nxt = -1
    return nxt



def start_point_cb(data):
    """
    This is the callback of start point set.
    param: data: PoseWithCovarianceStamped
    return: none
    """
    # x, y of the start_point
    start_x = data.pose.pose.position.x
    start_y = data.pose.pose.position.y

    # Orientation
    start_orientation = data.pose.pose.orientation
    (roll, poll, yaw) = euler_from_quaternion([start_orientation.x, start_orientation.y, 
                                               start_orientation.z, start_orientation.w])
    yaw = get_degree(yaw)

    # Current and next point.Int8()
    cur = Int8()
    nxt = Int8()
    cur.data = get_cur(start_x, start_y)
    nxt.data = get_nxt(cur.data, yaw)

    # Info
    # rospy.loginfo("cur:%d" % cur.data)
    # rospy.loginfo("nxt:%d" % nxt.data)

    # Publish
    cur_pub.publish(cur)
    nxt_pub.publish(nxt)


def main():
    """
        Main function starts
    """
    # Globals
    global cur_pub
    global nxt_pub
    global start_race_pub

    # Init node
    rospy.init_node("start_point_start_race_node")
    
    # Subscriber
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, start_point_cb, queue_size=1)
    
    # Publishers
    cur_pub = rospy.Publisher('/cur', Int8, queue_size=1)
    nxt_pub = rospy.Publisher('/nxt', Int8, queue_size=1)

    # Spin
    rospy.spin()
    """
        Main function end
    """


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
