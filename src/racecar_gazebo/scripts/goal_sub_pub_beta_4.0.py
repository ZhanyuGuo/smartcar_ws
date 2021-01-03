#!/usr/bin/env python
"""
    goal_sub_pub_beta_2.0.py
    2020.08.23
    Zhanyu Guo
"""
"""
    Imports
"""
import math
import random
import rospy
import tf
from std_msgs.msg import Bool, String, Int8
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

begin_flag = Bool()
reach_flag = Bool()
receive_flag = Bool()

begin_flag.data = False
reach_flag.data = True  # default True to begin the 1st pub
receive_flag.data = False

Goals = []
current_goal = PoseStamped()
current_goal.pose.position.x = 100
current_goal.pose.position.y = 100

cur = 0
nxt = 0
goal_dis = 2.2

choices = [[2, 9],
           [1, 10, 4],
           [4, 11],
           [2, 3, 5],
           [4, 6, 11],
           [5, 7],
           [6, 8, 10],
           [7, 9, 10],
           [1, 8],
           [2, 7, 8, 11],
           [3, 5, 10]]

dis = [[0, 5, -1, -1, -1, -1, -1, -1, 9, -1, -1],
       [5, 0, 6.24, 10, -1, -1, -1, -1, -1, 4.96, -1],
       [-1, 6.24, 0, 6.24, -1, -1, -1, -1, -1, -1, 1.78],
       [-1, 10, 6.24, 0, 5.1, -1, -1, -1, -1, -1, -1],
       [-1, -1, -1, 5.1, 0, 3.9, -1, -1, -1, -1, 6.98],
       [-1, -1, -1, -1, 3.9, 0, 7, -1, -1, -1, -1],
       [-1, -1, -1, -1, -1, 7, 0, 4.5, -1, 4.53, -1],
       [-1, -1, -1, -1, -1, -1, 4.5, 0, 3.5, 5.29, -1],
       [9, -1, -1, -1, -1, -1, -1, 3.5, 0, -1, -1],
       [-1, 4.96, -1, -1, -1, -1, 4.53, 5.29, -1, 0, 3],
       [-1, -1, 1.78, -1, 6.98, -1, -1, -1, -1, 3, 0]]

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


def set_xyzw(i, angle):
    """
    This function is used to set x, y, z, w in quaternion
    :param point: point to be set
    :param angle: angle
    :return: None
    """
    # Set x, y, z, w
    Goals[i].pose.orientation.x = get_quaternion(angle)[0]
    Goals[i].pose.orientation.y = get_quaternion(angle)[1]
    Goals[i].pose.orientation.z = get_quaternion(angle)[2]
    Goals[i].pose.orientation.w = get_quaternion(angle)[3]


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


def goal_sub_cb(data):
    """
    This is the callback of goal number subscriber
    param: data: String
    return: None
    """
    global cur
    global nxt
    # The list to store points
    # point_list = [ending_point, point_1, point_2, point_3, ...]
    point_list = list(map(int, data.data.split(' ')))

    # The ending point(destination)
    ending_point = point_list.pop(0)

    # Minimum distance default to infinite
    min_dis = float('inf')
    
    # Store shortest way
    shortest_way = []
    
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

    # find way flag
    find_way = False

    # error flag
    error = False

    # times of find way
    find_way_time = 0

    # main while, searching for a route
    while find_way_time < 10:
        pre = cur
        now = nxt
        way = [pre, now]
        tmp_dis = 0
        for i in range(20):
            count = len(choices[now - 1])
            p = choices[now - 1][random.randint(0, count - 1)]
            if p == pre:
                continue
            pre = now
            now = p
            tmp_dis += dis[now - 1][p - 1]  # distance from now to p
            way.append(p)
            if p == ending_point:
                for j in point_list:
                    if j not in way:
                        break
                else:
                    length = len(way)
                    for k in range(1, length - 1):
                        if way[k] == 2 and way[k - 1] in [3, 4] and way[k + 1] in [3, 4]:
                            error = True
                            break
                        if way[k] == 3 and way[k - 1] in [2, 4] and way[k + 1] in [2, 4]:
                            error = True
                            break
                        if way[k] == 4 and way[k - 1] in [2, 3] and way[k + 1] in [2, 3]:
                            error = True
                            break
                        if way[k] == 2 and way[k - 1] in [4, 10] and way[k + 1] in [4, 10]:
                            error = True
                            break
                        if way[k] == 2 and way[k - 1] in [3, 10] and way[k + 1] in [3, 10]:
                            error = True
                            break
                        if way[k] == 3 and way[k - 1] in [2, 11] and way[k + 1] in [2, 11]:
                            error = True
                            break
                        if way[k] == 10 and way[k - 1] in [2, 11] and way[k + 1] in [2, 11]:
                            error = True
                            break
                        if way[k] == 3 and way[k - 1] in [5, 11] and way[k + 1] in [5, 11]:
                            error = True
                            break
                        if way[k] == 5 and way[k - 1] in [4, 11] and way[k + 1] in [4, 11]:
                            error = True
                            break
                        if way[k] == 7 and way[k - 1] in [8, 10] and way[k + 1] in [8, 10]:
                            error = True
                            break
                        if way[k] == 8 and way[k - 1] in [7, 10] and way[k + 1] in [7, 10]:
                            error = True
                            break
                        if way[k] == 10 and way[k - 1] in [7, 8] and way[k + 1] in [7, 8]:
                            error = True
                            break
                        if way[k] == 11 and way[k - 1] in [3, 5] and way[k + 1] in [3, 5]:
                            error = True
                            break
                    if not error:
                        find_way = True
                        find_way_time += 1
                        if tmp_dis < min_dis:
                            min_dis = tmp_dis
                            shortest_way = way[:]
            if find_way:
                find_way = False
                break
            if error:
                error = False
                break

    shortest_way.pop(0)

    # Info the way before improvement
    for j in shortest_way:
        rospy.loginfo('%d ' % j)

    # Append the goals to Goals[] in order
    for k in shortest_way:
        Goals.append(point_pos[k - 1])
    
    l = len(Goals)
    for i in range(l - 1):
        if shortest_way[i] == 1:
            if shortest_way[i + 1] == 2:
                set_xyzw(i, 10)
            else:
                set_xyzw(i, -100)

        if shortest_way[i] == 2:
            if shortest_way[i + 1] == 1:
                set_xyzw(i, 180)
            elif shortest_way[i + 1] == 10:
                set_xyzw(i, -68)
            else:
                set_xyzw(i, 0)

        if shortest_way[i] == 3:
            if shortest_way[i + 1] == 4:
                set_xyzw(i, 49)
            else:
                set_xyzw(i, -131)

        if shortest_way[i] == 4:
            if shortest_way[i + 1] == 5:
                set_xyzw(i, -70)
            else:
                set_xyzw(i, 170)

        if shortest_way[i] == 5:
            if shortest_way[i + 1] == 4:
                set_xyzw(i, 90)
            elif shortest_way[i + 1] == 6:
                set_xyzw(i, -80)
            else:
                set_xyzw(i, 135)
    
        if shortest_way[i] == 6:
            if shortest_way[i + 1] == 5:
                set_xyzw(i, 80)
            else:
                set_xyzw(i, -170)

        if shortest_way[i] == 7:
            if shortest_way[i + 1] == 6:
                set_xyzw(i, 0)
            elif shortest_way[i + 1] == 10:
                set_xyzw(i, 112)
            else:
                set_xyzw(i, 180)

        if shortest_way[i] == 8:
            if shortest_way[i + 1] == 7:
                set_xyzw(i, 0)
            elif shortest_way[i + 1] == 10:
                set_xyzw(i, 49)
            else:
                set_xyzw(i, -170)

        if shortest_way[i] == 9:
            if shortest_way[i + 1] == 1:
                set_xyzw(i, 100)
            else:
                set_xyzw(i, -10)

        if shortest_way[i] == 10:
            if shortest_way[i + 1] == 8:
                set_xyzw(i, -110)
            elif shortest_way[i + 1] == 2:
                set_xyzw(i, 120)
            elif shortest_way[i + 1] == 7:
                set_xyzw(i, -49)
            else:
                set_xyzw(i, 49)

        if shortest_way[i] == 11:
            if shortest_way[i + 1] == 3:
                set_xyzw(i, 49)
            elif shortest_way[i + 1] == 5:
                set_xyzw(i, -30)
            else:
                set_xyzw(i, -160)
    
    if shortest_way[-1] == 1:
        if shortest_way[-2] == 9:
            set_xyzw(-1, 90)
        else:
            set_xyzw(-1, 180)

    if shortest_way[-1] == 2:
        if shortest_way[-2] == 1:
            set_xyzw(-1, 0)
        elif shortest_way[-2] == 4:
            set_xyzw(-1, 180)
        else:
            set_xyzw(-1, 120)

    if shortest_way[-1] == 3:
        if shortest_way[-2] == 4:
            set_xyzw(-1, -131)
        else:
            set_xyzw(-1, 49)

    if shortest_way[-1] == 4:
        if shortest_way[-2] == 5:
            set_xyzw(-1, 90)
        else:
            set_xyzw(-1, 0)

    if shortest_way[-1] == 5:
        if shortest_way[-2] == 4:
            set_xyzw(-1, -90)
        elif shortest_way[-2] == 6:
            set_xyzw(-1, 90)
        else:
            set_xyzw(-1, -30)

    if shortest_way[-1] == 6:
        if shortest_way[-2] == 5:
            set_xyzw(-1, -90)
        else:
            set_xyzw(-1, 0)

    if shortest_way[-1] == 7:
        if shortest_way[-2] == 6:
            set_xyzw(-1, 180)
        if shortest_way[-2] == 10:
            set_xyzw(-1, -49)
        else:
            set_xyzw(-1, 180)

    if shortest_way[-1] == 8:
        if shortest_way[-2] == 9:
            set_xyzw(-1, 0)
        elif shortest_way[-2] == 7:
            set_xyzw(-1, 180)
        else:
            set_xyzw(-1, 49)

    if shortest_way[-1] == 9:
        if shortest_way[-2] == 1:
            set_xyzw(-1, -90)
        else:
            set_xyzw(-1, 180)

    if shortest_way[-1] == 10:
        if shortest_way[-2] == 11:
                set_xyzw(i, -160)
        elif shortest_way[-2] == 7:
            set_xyzw(i, 120)
        elif shortest_way[-2] == 2:
            set_xyzw(i, -49)
        else:
            set_xyzw(i, 49)

    if shortest_way[-1] == 11:
        if shortest_way[-2] == 3:
            set_xyzw(-1, -120)
        elif shortest_way[-2] == 10:
            set_xyzw(-1, 49)
        else:
            set_xyzw(-1, 150)

    # finish receiving
    receive_flag.data = True


def listener_cb(_):
    """
    This is the callback of reach_flag_subscriber.
    param: data: MoveBaseActionResult
    return: None
    """
    global current_goal
    global goal_dis
    # Get TF from map to base_link
    if not rospy.is_shutdown():
        listener.waitForTransform('map','base_link',rospy.Time(0), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))

        # x and y of odometry, pose of the racecar
        odom_x = trans[0]
        odom_y = trans[1]

        # The distance between current goal and odom
        distance = get_distance(odom_x, odom_y,
                                current_goal.pose.position.x, current_goal.pose.position.y)

        # Info
        rospy.loginfo("distance to current goal:%.2f" % distance)
        if distance < goal_dis:
            # Info
            rospy.loginfo("Goal reached!...")

            # Already reached previous goal
            reach_flag.data = True
        else:
            pass

        # All condition
        if begin_flag.data and reach_flag.data and receive_flag.data:
            if not Goals:
                # Info
                rospy.loginfo('Final Goal Reach!...')
                rospy.signal_shutdown('Race over')
            else:
                # Get current goal
                current_goal = Goals.pop(0)

                # Publish current goal
                goal_pub.publish(current_goal)
                goal_pub.publish(current_goal)

                # Info
                rospy.loginfo('Pub next...')

            # Reach
            reach_flag.data = False

    
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
    rospy.Subscriber('/begin_flag', Bool, begin_sub_cb, queue_size=1)
    rospy.Subscriber('/points_num', String, goal_sub_cb, queue_size=1)
    rospy.Subscriber('/cur', Int8, cur_sub_cb, queue_size=1)
    rospy.Subscriber('/nxt', Int8, nxt_sub_cb, queue_size=1)

    # Publisher
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    # timer
    rospy.Timer(rospy.Duration(0.2), listener_cb)

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
