#!/usr/bin/env python
"""
    goal_sub_pub_beta_2.0.py
    2020.08.22
    Zhanyu Guo
"""
"""
    Imports
"""
import math
import rospy
import tf
from std_msgs.msg import Bool, String, Int8
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

begin_flag = Bool()
reach_flag = Bool()
receive_flag = Bool()
set_flag = Bool()

begin_flag.data = False
reach_flag.data = True  # default True to begin the 1st pub
receive_flag.data = False
set_flag.data = False

Goals = []
current_goal = PoseStamped()
current_goal.pose.position.x = 100
current_goal.pose.position.y = 100

cur = 0
nxt = 0

with open('/home/gzy/pose_.txt', 'r') as file_handle:
    pos_1 = PoseStamped()
    pos_2 = PoseStamped()
    pos_3 = PoseStamped()
    pos_4 = PoseStamped()
    pos_5 = PoseStamped()
    pos_6 = PoseStamped()
    pos_7 = PoseStamped()
    pos_8 = PoseStamped()
    pos_9 = PoseStamped()
    pos_10 = PoseStamped()
    pos_11 = PoseStamped()

    # point position list
    point_pos = [pos_1, pos_2, pos_3, pos_4, pos_5, pos_6, pos_7, pos_8, pos_9, pos_10, pos_11]

    for pos in point_pos:
        pos.pose.position.x = float(file_handle.readline().replace('\n', ''))
        pos.pose.position.y = float(file_handle.readline().replace('\n', ''))


def cur_sub_cb(data):
    """
    This function is the callback of /cur, gets the cur.
    data: Int8()
    return: none
    """
    global cur
    cur = data.data
    rospy.loginfo("current:%d" % cur)


def nxt_sub_cb(data):
    """
    This function is the callback of /nxt, gets the cur.
    data: Int8()
    return: none
    """
    global nxt
    nxt = data.data
    rospy.loginfo("next:%d" % nxt)

    
def get_distance(x1, y1, x2, y2):
    """
    This function gets distance between two points using x and y.
    param: x1: float
           y1: float
           x2: float
           y2: float
    return: dis: float
    """
    dis = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    return dis


def get_quaternion(angle):
    """
    This function is used to get quaternion from yaw:angle
    :param angle: yaw
    :return: q: quaternion
    """
    # From degrees to radians
    rad = math.radians(angle)

    # Get q(uaternion)
    q = quaternion_from_euler(0, 0, rad)
    return q


def set_xyzw(point, angle):
    """
    This function is used to set x, y, z, w in quaternion
    :param point: point to be set
    :param angle: angle
    :return: None
    """
    # Set x, y, z, w
    point_pos[point - 1].pose.orientation.x = get_quaternion(angle)[0]
    point_pos[point - 1].pose.orientation.y = get_quaternion(angle)[1]
    point_pos[point - 1].pose.orientation.z = get_quaternion(angle)[2]
    point_pos[point - 1].pose.orientation.w = get_quaternion(angle)[3]


def begin_sub_cb(data):
    """
    This is the callback of begin_flag_subscriber.
    param: data: PoseStamped
    return: None
    """
    # Get begin flag
    begin_flag.data = True

    # Info
    rospy.loginfo("Begin flag received!...")


# def reach_sub_cb(data):
#     pass


def goal_sub_cb(data):
    """
    This is the callback of goal number subscriber
    param: data: String
    return: None
    """
    global cur
    global nxt
    # The list to store points
    # point_list = [point_1, point_2, point_3, ...]
    point_list = list(map(int, data.data.split(' ')))

    # The ending point(destination)
    ending_point = point_list.pop(0)

    # Yaw defaults to 0
    yaw = 0
    q = get_quaternion(yaw)
    for pos in point_pos:
        pos.header.frame_id = 'map'
        pos.pose.position.z = 0.0
        pos.pose.orientation.x = q[0]
        pos.pose.orientation.y = q[1]
        pos.pose.orientation.z = q[2]
        pos.pose.orientation.w = q[3]

    # The list to store a fix way
    way_ls = [1, 2, 10, 7, 6, 5, 4, 3, 11, 10, 8, 9,
              1, 2, 10, 7, 6, 5, 4, 3, 11, 10, 8, 9,
              1, 2, 10, 7, 6, 5, 4, 3, 11, 10, 8, 9]

    # Reverse way_ls to sl_yaw
    sl_yaw = way_ls[::-1]

    # The list to store the way
    way = []

    # Special cases
    if cur == 2 and nxt == 4:
        way.append(cur)
        cur = 4
        nxt = 5
    elif cur == 5 and nxt == 11:
        way.append(cur)
        cur = 11
        nxt = 10
    elif cur == 11 and nxt == 5:
        way.append(cur)
        cur = 5
        nxt = 6
    elif cur == 7 and nxt == 8:
        way.append(cur)
        cur = 8
        nxt = 9
    elif cur == 8 and nxt == 7:
        way.append(cur)
        cur = 7
        nxt = 6
    else:
        pass

    # Flags
    find_cur = False
    find_next = False
    find_way = False
    reverse_flag = False

    # Check
    for i in way_ls:
        if find_way:
            break

        if i == ending_point and find_next:
            for j in point_list:
                if j not in way:
                    break

            else:
                find_way = True

        if find_next:
            way.append(i)

        if find_cur:
            if i == nxt:
                way.append(i)
                find_next = True
                find_cur = False
            else:
                way.pop(0)
                find_cur = False

        if i == cur and not find_cur and not find_next:
            find_cur = True
            way.append(i)

    # Reverse
    if len(way) < 2:
        way = []
        reverse_flag = True
        find_cur = False
        find_next = False
        find_way = False
        for i in sl_yaw:
            if find_way:
                break

            if i == ending_point and find_next:
                for j in point_list:
                    if j not in way:
                        break

                else:
                    find_way = True

            if find_next:
                way.append(i)

            if find_cur:
                if i == nxt:
                    way.append(i)
                    find_next = True
                    find_cur = False
                else:
                    way.pop(0)
                    find_cur = False

            if i == cur and not find_cur and not find_next:
                find_cur = True
                way.append(i)

    # improve the goal's position's yaw
    if not reverse_flag:
        set_xyzw(1, 0)
        set_xyzw(2, -68)
        set_xyzw(3, -131)
        set_xyzw(4, 180)
        set_xyzw(5, 90)
        set_xyzw(6, 90)
        set_xyzw(7, 0)
        set_xyzw(8, 180)
        set_xyzw(9, 90)
        set_xyzw(10, -100)
        set_xyzw(11, -131)
    else:
        set_xyzw(1, -90)
        set_xyzw(2, 180)
        set_xyzw(3, 49)
        set_xyzw(4, -90)
        set_xyzw(5, -90)
        set_xyzw(6, 180)
        set_xyzw(7, 112)
        set_xyzw(8, 49)
        set_xyzw(9, 0)
        set_xyzw(10, 80)
        set_xyzw(11, 49)

    # Remove the starting_point
    way.pop(0)

    # Info the way before improvement
    for j in way:
        rospy.loginfo('%d ' % j)

    # Append the goals to Goals[] in order
    for k in way:
        Goals.append(point_pos[k - 1])

    # finish receiving
    receive_flag.data = True


def publisher_cb(_):
    """
    This is the callback of a timer with 20Hz, checking if it is time to publish goals
    param: None
    return: None
    """
    global current_goal
    # all condition
    if begin_flag.data and reach_flag.data and receive_flag.data:
        if not Goals:
            # Info
            rospy.loginfo('Final Goal Reach!...')
            rospy.signal_shutdown('Over')
        else:
            # Get current goal
            current_goal = Goals.pop(0)

            # Publish current goal
            goal_pub.publish(current_goal)

            # Info
            rospy.loginfo('Pub next...')

        # Reach
        reach_flag.data = False


def listener_cb(_):
    """
    This is the callback of reach_flag_subscriber.
    param: data: MoveBaseActionResult
    return: None
    """
    global current_goal
    if not rospy.is_shutdown():
        listener.waitForTransform('map','base_link',rospy.Time(0), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        odom_x = trans[0]
        odom_y = trans[1]

        # The distance between current goal and odom
        distance = get_distance(odom_x, odom_y,
                                current_goal.pose.position.x, current_goal.pose.position.y)

        # Info
        rospy.loginfo("distance to current goal:%.2f" % distance)
        if distance < 2.0 and not set_flag.data:
            # Info
            rospy.loginfo("Goal reached!...")

            # Already reached previous goal
            reach_flag.data = True
            set_flag.data = True
        elif distance > 2.0:
            set_flag.data = False
        else:
            pass

    
def main():
    """
        Main function starts
    """
    # global
    global goal_pub
    global listener

    # Init node
    rospy.init_node("goal_sub_pub_node")

    # TF listener
    listener = tf.TransformListener()

    # Subscribers
    # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, reach_sub_cb, queue_size=1)
    rospy.Subscriber('/begin_flag', Bool, begin_sub_cb, queue_size=1)
    rospy.Subscriber('/points_num', String, goal_sub_cb, queue_size=1)
    rospy.Subscriber('/cur', Int8, cur_sub_cb, queue_size=1)
    rospy.Subscriber('/nxt', Int8, nxt_sub_cb, queue_size=1)

    # Publisher
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    # timer
    rospy.Timer(rospy.Duration(0.5), listener_cb)
    rospy.Timer(rospy.Duration(0.5), publisher_cb)

    # spin
    rospy.spin()
    """
        Main function ends
    """


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
