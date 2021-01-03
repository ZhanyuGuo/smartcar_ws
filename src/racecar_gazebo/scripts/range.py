import itertools


def main():
    """
    main function
    :return: none
    """
    point_ls = list(range(1, 12))  # 总共11个点

    # starting_point = int(input('输入起点：'))  # 输入起点
    starting_point = 5  # 输入起点
    # ending_point = int(input('输入终点：'))  # 输入终点
    ending_point = 3  # 输入终点

    point_ls.remove(starting_point)  # 列表中去除起点
    point_ls.remove(ending_point)  # 列表中去除终点

    # goal_str = input('输入必须经过的点：(以空格间隔)')
    # goal_ls = list(map(int, goal_str.split(' ')))
    goal_ls = [1, 2, 4, 6, 9]  # 必须经过点的列表

    count = len(goal_ls)  # 计算经过点的个数

    # 两点之间最短距离矩阵
    dis_matrix = [[0, 4, 9, 13, 15, 18, 11, 11, 8, 8, 10],
                  [4, 0, 5, 9, 11, 14, 7, 8, 11, 4, 6],
                  [9, 5, 0, 5, 6, 9, 6, 7, 10, 3, 1],
                  [13, 9, 5, 0, 4, 7, 11, 12, 15, 8, 6],
                  [15, 11, 6, 4, 0, 3, 10, 11, 14, 7, 5],
                  [18, 14, 9, 7, 3, 0, 7, 11, 14, 10, 8],
                  [11, 7, 6, 11, 10, 7, 0, 4, 7, 3, 5],
                  [11, 8, 7, 12, 11, 11, 4, 0, 3, 4, 6],
                  [8, 11, 10, 15, 14, 14, 7, 3, 0, 7, 9],
                  [8, 4, 3, 8, 7, 10, 3, 4, 7, 0, 2],
                  [10, 6, 1, 6, 5, 8, 5, 6, 9, 2, 0]]

    min_dis = float('inf')  # 将最小距离初始化为inf(无穷)
    shortest_way = []
    for possibility in itertools.permutations(goal_ls, count):
        tmp = 0  # 计算临时值
        for pos in range(count - 1):
            tmp += dis_matrix[possibility[pos] - 1][possibility[pos + 1] - 1]
            pass
        if tmp < min_dis:
            min_dis = tmp
            shortest_way = list(possibility)
            pass
        pass

    print('起点：{};\n终点：{};\n最短距离为：{};\n路径是：{}。'.format(starting_point, ending_point, min_dis, shortest_way))
    pass


if __name__ == '__main__':
    main()
    pass
