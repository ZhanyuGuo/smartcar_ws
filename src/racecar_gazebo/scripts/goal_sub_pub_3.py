#!/usr/bin/env python

"""
imports
"""
import itertools
import math
import rospy
from std_msgs.msg import Bool, String
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped, PointStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

begin_flag = Bool()
reach_flag = Bool()
receive_flag = Bool()
forward_flag = Bool()
begin_flag.data = False
reach_flag.data = True  # default True to begin the 1st pub
receive_flag.data = False
forward_flag.data = True  # default True to begin the 1st pub
shortest_way = []
Goals = []
count = 0
i = 0

pos_1 = PoseStamped()
pos_1.pose.position.x = -0.247200012207
pos_1.pose.position.y = -0.110009670258

pos_2 = PoseStamped()
pos_2.pose.position.x = 4.46373748779
pos_2.pose.position.y = -0.110009670258

pos_3 = PoseStamped()
pos_3.pose.position.x = 9.34290027618
pos_3.pose.position.y = -1.0710849762

pos_4 = PoseStamped()
pos_4.pose.position.x = 14.3318080902
pos_4.pose.position.y = -0.306130886078

pos_5 = PoseStamped()
pos_5.pose.position.x = 14.3978767395
pos_5.pose.position.y = -4.79310750961

pos_6 = PoseStamped()
pos_6.pose.position.x = 14.3670244217
pos_6.pose.position.y = -8.97793579102

pos_7 = PoseStamped()
pos_7.pose.position.x = 7.39731168747
pos_7.pose.position.y = -8.93243026733

pos_8 = PoseStamped()
pos_8.pose.position.x = 3.11276102066
pos_8.pose.position.y = -8.90151596069

pos_9 = PoseStamped()
pos_9.pose.position.x = -0.272539615631
pos_9.pose.position.y = -8.678647995

pos_10 = PoseStamped()
pos_10.pose.position.x = 6.33073949814
pos_10.pose.position.y = -4.44014024734

pos_11 = PoseStamped()
pos_11.pose.position.x = 8.44114398956
pos_11.pose.position.y = -2.06265521049

# point position list
point_pos = [
    pos_1,
    pos_2,
    pos_3,
    pos_4,
    pos_5,
    pos_6,
    pos_7,
    pos_8,
    pos_9,
    pos_10,
    pos_11,
]


def get_quaternion(angle):
    """
    This function is used to get quaternion from yaw:angle
    :param angle: yaw
    :return: q: quaternion
    """
    rad = math.radians(angle)
    q = quaternion_from_euler(0, 0, rad)
    return q


def set_xyzw(point, angle):
    """
    This function is used to set x, y, z, w in quaternion
    :param point: point to be set
    :param angle: angle
    :return: None
    """
    point_pos[point - 1].pose.orientation.x = get_quaternion(angle)[0]
    point_pos[point - 1].pose.orientation.y = get_quaternion(angle)[1]
    point_pos[point - 1].pose.orientation.z = get_quaternion(angle)[2]
    point_pos[point - 1].pose.orientation.w = get_quaternion(angle)[3]


def forward():
    global i
    global count

    orient = point_pos[shortest_way[i] - 1].pose.orientation

    (r, p, y) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
    rospy.loginfo("rad = %f" % y)
    rospy.loginfo("point = %d" % shortest_way[i])
    dl = 0.1
    dx = dl * math.cos(y)
    dy = dl * math.sin(y)
    forward_point = PoseStamped()
    forward_point.pose.position.x = point_pos[shortest_way[i] - 1].pose.position.x + dl
    forward_point.pose.position.y = point_pos[shortest_way[i] - 1].pose.position.y
    forward_point.pose.position.z = point_pos[shortest_way[i] - 1].pose.position.z
    forward_point.pose.orientation.x = point_pos[shortest_way[i] - 1].pose.orientation.x
    forward_point.pose.orientation.y = point_pos[shortest_way[i] - 1].pose.orientation.y
    forward_point.pose.orientation.z = point_pos[shortest_way[i] - 1].pose.orientation.z
    forward_point.pose.orientation.w = point_pos[shortest_way[i] - 1].pose.orientation.w

    i += 1
    goal_pub.publish(forward_point)


def begin_sub_cb(data):
    """
    This is the callback of begin_flag_subscriber.
    param: data: PoseStamped
    return: None
    """
    # get flag
    begin_flag.data = True

    # append the final goal
    Goals.append(data)

    # info
    rospy.loginfo("Begin flag received!...")


def reach_sub_cb(data):
    """
    This is the callback of reach_flag_subscriber.
    param: data: MoveBaseActionResult
    return: None
    """
    # info
    rospy.loginfo("Goal reached!...")

    # already reached previous goal but need to go forward
    if not forward_flag.data:
        forward()
        forward_flag.data = True
    else:
        reach_flag.data = True
    # reach_flag.data = True


def goal_sub_cb(data):
    """
    This is the callback of goal number subscriber
    param: data: String
    return: None
    """
    global count
    global shortest_way

    # store the goals' numbers
    goal_ls = list(map(int, data.data.split(" ")))

    # starting point and ending point
    starting_point = goal_ls.pop(0)
    ending_point = goal_ls.pop(-1)

    # counts of the numbers
    count = len(goal_ls)

    # min distance matrix
    dis_matrix = [
        [0, 5, 11.24, 15, 16.97, 20.87, 14.49, 12.5, 9, 9.96, 9.99],
        [5, 0, 6.24, 10, 11.97, 15.87, 9.49, 10.25, 13.75, 4.96, 4.99],
        [11.24, 6.24, 0, 6.24, 8.76, 12.66, 6.34, 7.1, 10.6, 1.81, 1.78],
        [15, 10, 6.24, 0, 5.1, 9, 12.58, 13.34, 16.84, 8.05, 8.02],
        [16.97, 11.97, 6.24, 5.1, 0, 3.9, 10.9, 12.3, 15.8, 7.01, 6.98],
        [20.87, 15.87, 12.66, 9, 3.9, 0, 7, 11.5, 15, 10.91, 10.88],
        [14.49, 9.49, 6.34, 12.58, 10.9, 7, 0, 4.5, 8, 4.53, 4.56],
        [12.5, 10.25, 7.1, 13.34, 12.3, 11.5, 4.5, 0, 3.5, 5.29, 5.32],
        [9, 13.75, 10.6, 16.84, 15.8, 15, 8, 3.5, 0, 8.79, 8.82],
        [9.96, 4.96, 1.81, 8.05, 7.01, 10.91, 4.53, 5.29, 8.79, 0, 3],
        [9.99, 4.99, 1.78, 8.02, 6.98, 10.88, 4.56, 5.32, 8.82, 3, 0],
    ]

    z = 0
    q = get_quaternion(z)
    for pos in point_pos:
        pos.header.frame_id = "map"
        pos.pose.position.z = 0.0
        pos.pose.orientation.x = q[0]
        pos.pose.orientation.y = q[1]
        pos.pose.orientation.z = q[2]
        pos.pose.orientation.w = q[3]

    # min distance inited as infinite
    min_dis = float("inf")

    # store the shortest_way
    shortest_way = []

    # range all possibility
    for possibility in itertools.permutations(goal_ls, count):
        # temporary value
        tmp = 0

        # distance from starting point to 1st point
        tmp += dis_matrix[starting_point - 1][possibility[0] - 1]
        for pos in range(count - 1):
            # distance from x to x + 1
            tmp += dis_matrix[possibility[pos] - 1][possibility[pos + 1] - 1]

        # distance from final point to ending point
        tmp += dis_matrix[possibility[-1] - 1][ending_point - 1]

        # get the min distance and shortest way
        if tmp < min_dis:
            min_dis = tmp
            shortest_way = list(possibility)

    # improve the goal's position's yaw
    for p in shortest_way:
        index = shortest_way.index(p)
        if index == 0:
            if p == 1:
                if starting_point in [8, 9]:
                    set_xyzw(p, 0)
                else:
                    set_xyzw(p, -90)
            if p == 2:
                if starting_point in [3, 4, 7, 11]:
                    set_xyzw(p, 180)
                elif starting_point == 1:
                    set_xyzw(p, 0)
                else:
                    set_xyzw(p, 112)
            if p == 3:
                if starting_point in [1, 2, 4]:
                    set_xyzw(p, -131)
                else:
                    set_xyzw(p, 49)
            if p == 4:
                if starting_point in [5, 6]:
                    set_xyzw(p, 180)
                else:
                    set_xyzw(p, -90)
            if p == 5:
                if starting_point in [6, 7]:
                    set_xyzw(p, 90)
                elif starting_point == 4:
                    set_xyzw(p, -90)
                else:
                    set_xyzw(p, 0)
            if p == 6:
                if starting_point in [7, 8, 9]:
                    set_xyzw(p, 90)
                else:
                    set_xyzw(p, 180)
            if p == 7:
                if starting_point in [5, 6]:
                    set_xyzw(p, 180)
                elif starting_point in [8, 9]:
                    set_xyzw(p, 0)
                else:
                    set_xyzw(p, -90)
            if p == 8:
                if starting_point in [1, 9]:
                    set_xyzw(p, 0)
                elif starting_point in [6, 7]:
                    set_xyzw(p, 180)
                else:
                    set_xyzw(p, -131)
            if p == 9:
                if starting_point == 1:
                    set_xyzw(p, 0)
                else:
                    set_xyzw(p, 90)
            if p == 10:
                if starting_point in [1, 2]:
                    set_xyzw(p, -68)
                elif starting_point == 7:
                    set_xyzw(p, 112)
                elif starting_point in [8, 9]:
                    set_xyzw(p, 49)
                else:
                    set_xyzw(p, -131)
            if p == 11:
                if starting_point in [3, 4]:
                    set_xyzw(p, -131)
                elif starting_point in [5, 6]:
                    set_xyzw(p, 154)
                else:
                    set_xyzw(p, 49)
        else:
            if p == 1:
                if shortest_way[index - 1] in [8, 9]:
                    set_xyzw(p, 0)
                else:
                    set_xyzw(p, -90)
            if p == 2:
                if shortest_way[index - 1] in [3, 4, 7, 11]:
                    set_xyzw(p, 180)
                elif shortest_way[index - 1] == 1:
                    set_xyzw(p, 0)
                else:
                    set_xyzw(p, 112)
            if p == 3:
                if shortest_way[index - 1] in [1, 2, 4]:
                    set_xyzw(p, -131)
                else:
                    set_xyzw(p, 49)
            if p == 4:
                if shortest_way[index - 1] in [5, 6]:
                    set_xyzw(p, 180)
                else:
                    set_xyzw(p, -90)
            if p == 5:
                if shortest_way[index - 1] in [6, 7]:
                    set_xyzw(p, 90)
                elif shortest_way[index - 1] == 4:
                    set_xyzw(p, -90)
                else:
                    set_xyzw(p, 0)
            if p == 6:
                if shortest_way[index - 1] in [7, 8, 9]:
                    set_xyzw(p, 90)
                else:
                    set_xyzw(p, 180)
            if p == 7:
                if shortest_way[index - 1] in [5, 6]:
                    set_xyzw(p, 180)
                elif shortest_way[index - 1] in [8, 9]:
                    set_xyzw(p, 0)
                else:
                    set_xyzw(p, -90)
            if p == 8:
                if shortest_way[index - 1] in [1, 9]:
                    set_xyzw(p, 0)
                elif shortest_way[index - 1] in [6, 7]:
                    set_xyzw(p, 180)
                else:
                    set_xyzw(p, -131)
            if p == 9:
                if shortest_way[index - 1] == 1:
                    set_xyzw(p, 0)
                else:
                    set_xyzw(p, 90)
            if p == 10:
                if shortest_way[index - 1] in [1, 2]:
                    set_xyzw(p, -68)
                elif shortest_way[index - 1] == 7:
                    set_xyzw(p, 112)
                elif shortest_way[index - 1] in [8, 9]:
                    set_xyzw(p, 49)
                else:
                    set_xyzw(p, -131)
            if p == 11:
                if shortest_way[index - 1] in [3, 4]:
                    set_xyzw(p, -131)
                elif shortest_way[index - 1] in [5, 6]:
                    set_xyzw(p, 154)
                else:
                    set_xyzw(p, 49)

    # append the goals to Goals[] in order
    for k in range(count):
        Goals.append(point_pos[shortest_way[k] - 1])

    for j in shortest_way:
        rospy.loginfo("%d " % j)
    # finish receiving
    receive_flag.data = True


def publisher_cb(_):
    """
    This is the callback of a timer with 20Hz, checking if it is time to publish goals
    param: None
    return: None
    """
    # all condition
    if begin_flag.data and reach_flag.data and receive_flag.data and forward_flag.data:
        if not Goals:
            rospy.loginfo("Final Goal Reach!...")
        else:
            goal_pub.publish(Goals.pop(0))
        reach_flag.data = False
        forward_flag.data = False


def main():
    """
    main function starts
    """
    # global
    global goal_pub

    # init node
    rospy.init_node("goal_sub_pub_node")

    # Subscribers
    rospy.Subscriber(
        "/move_base/result", MoveBaseActionResult, reach_sub_cb, queue_size=1
    )
    rospy.Subscriber(
        "/move_base_simple/my_goal", PoseStamped, begin_sub_cb, queue_size=1
    )
    rospy.Subscriber("/points_num", String, goal_sub_cb, queue_size=1)

    # Publisher
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

    # timer
    rospy.Timer(rospy.Duration(0.5), publisher_cb)

    # spin
    rospy.spin()
    """
    main function end
    """


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
