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
from tf.transformations import quaternion_from_euler

begin_flag = Bool()
reach_flag = Bool()
receive_flag = Bool()

begin_flag.data = False
reach_flag.data = True  # default True to begin the 1st pub
receive_flag.data = False

Goals = []

pos_1 = PoseStamped()
pos_1.pose.position.x = 0.0
pos_1.pose.position.y = -0.7

pos_2 = PoseStamped()
pos_2.pose.position.x = 5.5
pos_2.pose.position.y = -0.7

pos_3 = PoseStamped()
pos_3.pose.position.x = 9.25
pos_3.pose.position.y = -1.81

pos_4 = PoseStamped()
pos_4.pose.position.x = 15.1
pos_4.pose.position.y = -0.7

pos_5 = PoseStamped()
pos_5.pose.position.x = 15.1
pos_5.pose.position.y = -5.0

pos_6 = PoseStamped()
pos_6.pose.position.x = 15.1
pos_6.pose.position.y = -8.8

pos_7 = PoseStamped()
pos_7.pose.position.x = 8.1
pos_7.pose.position.y = -8.8

pos_8 = PoseStamped()
pos_8.pose.position.x = 3.5
pos_8.pose.position.y = -8.8

pos_9 = PoseStamped()
pos_9.pose.position.x = 0.0
pos_9.pose.position.y = -8.8

pos_10 = PoseStamped()
pos_10.pose.position.x = 6.86
pos_10.pose.position.y = -4.42

pos_11 = PoseStamped()
pos_11.pose.position.x = 8.5
pos_11.pose.position.y = -2.7

# point position list
point_pos = [pos_1, pos_2, pos_3, pos_4, pos_5, pos_6, pos_7, pos_8, pos_9, pos_10, pos_11]


def get_quaternion(angle):
    """
    This function is used to get quaternion from yaw(angle).
    :param angle: yaw
    :return: q: quaternion
    """
    # From angle to rad
    rad = math.radians(angle)

    # Get q
    q = quaternion_from_euler(0, 0, rad)
    return q


def set_xyzw(point, angle):
    """
    This function is used to set x, y, z, w in quaternion
    :param point: point to be set
    :param angle: angle
    :return: None
    """
    # set x, y, z, w
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
    # Get flag
    begin_flag.data = True

    # Append the final goal
    Goals.append(data)

    # Info
    rospy.loginfo("Begin flag received!...")


def reach_sub_cb(data):
    """
    This is the callback of reach_flag_subscriber.
    param: data: MoveBaseActionResult
    return: None
    """
    # Info
    rospy.loginfo("Goal reached!...")

    # Already reached previous goal
    reach_flag.data = True


def goal_sub_cb(data):
    """
    This is the callback of goal numbers subscriber.
    param: data: String, goal numbers
    return: None
    """
    # The list stores the goals' numbers
    goal_ls = list(map(int, data.data.split(' ')))

    # starting point and ending point
    starting_point = goal_ls.pop(0)
    ending_point = goal_ls.pop(-1)

    # If goal_ls is empty, it is no need to continue
    if not goal_ls:
        # Info
        rospy.loginfo('No middle goal!...')
        return

    # The counts of middle goals' numbers
    count = len(goal_ls)

    # Minimum distance matrix[11 * 11]
    dis_matrix = [[0, 5, 11.24, 15, 16.97, 20.87, 14.49, 12.5, 9, 9.96, 9.99],
                  [5, 0, 6.24, 10, 11.97, 15.87, 9.49, 10.25, 13.75, 4.96, 4.99],
                  [11.24, 6.24, 0, 6.24, 8.76, 12.66, 6.34, 7.1, 10.6, 1.81, 1.78],
                  [15, 10, 6.24, 0, 5.1, 9, 12.58, 13.34, 16.84, 8.05, 8.02],
                  [16.97, 11.97, 6.24, 5.1, 0, 3.9, 10.9, 12.3, 15.8, 7.01, 6.98],
                  [20.87, 15.87, 12.66, 9, 3.9, 0, 7, 11.5, 15, 10.91, 10.88],
                  [14.49, 9.49, 6.34, 12.58, 10.9, 7, 0, 4.5, 8, 4.53, 4.56],
                  [12.5, 10.25, 7.1, 13.34, 12.3, 11.5, 4.5, 0, 3.5, 5.29, 5.32],
                  [9, 13.75, 10.6, 16.84, 15.8, 15, 8, 3.5, 0, 8.79, 8.82],
                  [9.96, 4.96, 1.81, 8.05, 7.01, 10.91, 4.53, 5.29, 8.79, 0, 3],
                  [9.99, 4.99, 1.78, 8.02, 6.98, 10.88, 4.56, 5.32, 8.82, 3, 0]]

    # Minimum way matrix[11 * 11 * x]
    way_matrix = [
        [[1], [1, 2], [1, 2, 3], [1, 2, 4], [1, 2, 10, 11, 5], [1, 2, 10, 7, 6], [1, 2, 10, 7], [1, 9, 8], [1, 9],
         [1, 2, 10], [1, 2, 10, 11]],
        [[2, 1], [2], [2, 3], [2, 4], [2, 10, 11, 5], [2, 10, 7, 6], [2, 10, 7], [2, 10, 8], [2, 10, 8, 9], [2, 10],
         [2, 10, 11]],
        [[3, 2, 1], [3, 2], [3], [3, 4], [3, 11, 5], [3, 11, 5, 6], [3, 11, 10, 7], [3, 11, 10, 8], [3, 11, 10, 8, 9],
         [3, 11, 10], [3, 11]],
        [[4, 2, 1], [4, 2], [4, 3], [4], [4, 5], [4, 5, 6], [4, 3, 11, 10, 7], [4, 3, 11, 10, 8], [4, 3, 11, 10, 8, 9],
         [4, 3, 11, 10], [4, 3, 11]],
        [[5, 11, 3, 2, 1], [5, 11, 3, 2], [5, 11, 3], [5, 4], [5], [5, 6], [5, 6, 7], [5, 11, 10, 8], [5, 11, 10, 8, 9],
         [5, 11, 10], [5, 11]],
        [[6, 5, 11, 3, 2, 1], [6, 5, 11, 3, 2], [6, 5, 11, 3], [6, 5, 4], [6, 5], [6], [6, 7], [6, 7, 8], [6, 7, 8, 9],
         [6, 7, 10], [6, 5, 11]],
        [[7, 10, 2, 1], [7, 10, 2], [7, 10, 11, 3], [7, 10, 11, 3, 4], [7, 10, 11, 5], [7, 6], [7], [7, 8], [7, 8, 9],
         [7, 10], [7, 10, 11]],
        [[8, 9, 1], [8, 10, 2], [8, 10, 11, 3], [8, 10, 11, 3, 4], [8, 10, 11, 5], [8, 7, 6], [8, 7], [8], [8, 9],
         [8, 10], [8, 10, 11]],
        [[9, 1], [9, 8, 10, 2], [9, 8, 10, 11, 3], [9, 8, 10, 11, 3, 4], [9, 8, 10, 11, 5], [9, 8, 7, 6], [9, 8, 7],
         [9, 8], [9], [9, 8, 10], [9, 8, 10, 11]],
        [[10, 2, 1], [10, 2], [10, 11, 3], [10, 11, 3, 4], [10, 11, 5], [10, 7, 6], [10, 7], [10, 8], [10, 8, 9], [10],
         [10, 11]],
        [[11, 3, 2, 1], [11, 3, 2], [11, 3], [11, 3, 4], [11, 5], [11, 5, 6], [11, 10, 7], [11, 10, 8], [11, 10, 8, 9],
         [11, 10], [11]]]

    # All points' yaw default to 0
    yaw = 0
    q = get_quaternion(yaw)
    for pos in point_pos:
        pos.header.frame_id = 'map'
        pos.pose.position.z = 0.0
        pos.pose.orientation.x = q[0]
        pos.pose.orientation.y = q[1]
        pos.pose.orientation.z = q[2]
        pos.pose.orientation.w = q[3]

    # Minimum distance default to infinite
    min_dis = float('inf')

    # The list stores the shortest_way
    shortest_way = []

    # Traverse all possibilities
    for possibility in itertools.permutations(goal_ls, count):
        # Temporary value
        tmp = 0

        # The distance from starting point to the 1st goal
        tmp += dis_matrix[starting_point - 1][possibility[0] - 1]
        for pos in range(count - 1):
            # The distance from x to x + 1
            tmp += dis_matrix[possibility[pos] - 1][possibility[pos + 1] - 1]

        # The distance from last goal to ending point
        tmp += dis_matrix[possibility[-1] - 1][ending_point - 1]

        # Get the min distance and shortest way
        if tmp < min_dis:
            min_dis = tmp
            shortest_way = list(possibility)

    a = 1
    for j in shortest_way:
        rospy.loginfo('%d point:%d' % (a, j))
        a += 1
        
    rospy.loginfo('------------------------------------')
    # Copy the original shortest_way to _fix
    shortest_way_fix = shortest_way[:]
    shortest_way_fix.append(ending_point)

    # Remove flag
    rm_flag = False
    # shortest_way = [2, 7]
    # shortest_way_fix = [2, 7, 6]
    # Traverse every point in shortest_way_fix
    for point in shortest_way_fix:
        # The shortest way from starting point to point x
        way_ls = way_matrix[starting_point - 1][point - 1][:-1]
        for p in shortest_way:
            # In way list and before it
            if p in way_ls and shortest_way_fix.index(p) < shortest_way_fix.index(point):
                # Set to -1, remove later
                shortest_way[shortest_way.index(p)] = -1
                rm_flag = True
        if rm_flag:
            shortest_way.remove(-1)
            rm_flag = False

    # If shortest_way is empty, it is no need to continue
    if not shortest_way:
        # Info
        rospy.loginfo('No middle goal!...')
        return

    count = len(shortest_way)

    # Improve the goal's position's yaw
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

    # Append the goals to Goals[] in order
    for k in range(count):
        Goals.append(point_pos[shortest_way[k] - 1])

    a = 1
    for j in shortest_way:
        rospy.loginfo('%d point:%d' % (a, j))
        a += 1

    # Finish receiving
    receive_flag.data = True


def publisher_cb(_):
    """
    This is the callback of a timer with 2Hz, checking if it is time to publish goals.
    param: None
    return: None
    """
    # All condition to be satisfied
    if begin_flag.data and reach_flag.data and receive_flag.data:
        if not Goals:
            rospy.loginfo('Final Goal Reach!...')
        else:
            goal_pub.publish(Goals.pop(0))
        reach_flag.data = False


def main():
    """
    Main function starts.
    """
    # Global
    global goal_pub

    # Init node
    rospy.init_node("goal_sub_pub_node")

    # Subscribers
    rospy.Subscriber('/points_num', String, goal_sub_cb, queue_size=1)
    rospy.Subscriber('/move_base_simple/my_goal', PoseStamped, begin_sub_cb, queue_size=1)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, reach_sub_cb, queue_size=1)

    # Publisher
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    # Timer: 2Hz check whether to`publish goals
    rospy.Timer(rospy.Duration(0.5), publisher_cb)

    # Spin
    rospy.spin()
    """
    Main function ends.
    """


if __name__ == "__main__":
    """
    Program starts here.
    """
    try:
        main()
    except rospy.ROSInterruptException:
        pass
