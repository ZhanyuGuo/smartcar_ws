import random

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


def main():
    cur = int(input('cur:'))
    nxt = int(input('nxt:'))
    point_ls = list(map(int, input('point list(split by ' '):').split(' ')))
    ending_point = point_ls.pop(0)
    min_dis = float('inf')
    shortest_way = []
    find_way = False
    error = False
    find_way_time = 0
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
                pass
            pre = now
            now = p
            tmp_dis += dis[now - 1][p - 1]  # now -> p
            way.append(p)
            if p == ending_point:
                for j in point_ls:
                    if j not in way:
                        break
                        pass
                    pass
                else:
                    length = len(way)
                    for k in range(1, length - 1):
                        if way[k] == 2 and way[k - 1] in [3, 4] and way[k + 1] in [3, 4]:
                            error = True
                            break
                            pass
                        if way[k] == 3 and way[k - 1] in [2, 4] and way[k + 1] in [2, 4]:
                            error = True
                            break
                            pass
                        if way[k] == 4 and way[k - 1] in [2, 3] and way[k + 1] in [2, 3]:
                            error = True
                            break
                            pass
                        if way[k] == 2 and way[k - 1] in [4, 10] and way[k + 1] in [4, 10]:
                            error = True
                            break
                            pass
                        if way[k] == 2 and way[k - 1] in [3, 10] and way[k + 1] in [3, 10]:
                            error = True
                            break
                            pass
                        if way[k] == 3 and way[k - 1] in [2, 11] and way[k + 1] in [2, 11]:
                            error = True
                            break
                            pass
                        if way[k] == 10 and way[k - 1] in [2, 11] and way[k + 1] in [2, 11]:
                            error = True
                            break
                            pass
                        if way[k] == 3 and way[k - 1] in [5, 11] and way[k + 1] in [5, 11]:
                            error = True
                            break
                            pass
                        if way[k] == 5 and way[k - 1] in [4, 11] and way[k + 1] in [4, 11]:
                            error = True
                            break
                            pass
                        if way[k] == 7 and way[k - 1] in [8, 10] and way[k + 1] in [8, 10]:
                            error = True
                            break
                            pass
                        if way[k] == 8 and way[k - 1] in [7, 10] and way[k + 1] in [7, 10]:
                            error = True
                            break
                            pass
                        if way[k] == 10 and way[k - 1] in [7, 8] and way[k + 1] in [7, 8]:
                            error = True
                            break
                            pass
                        if way[k] == 11 and way[k - 1] in [3, 5] and way[k + 1] in [3, 5]:
                            error = True
                            break
                            pass
                        pass
                    if not error:
                        find_way = True
                        find_way_time += 1
                        if tmp_dis < min_dis:
                            min_dis = tmp_dis
                            shortest_way = way[:]
                            pass
                        pass
                    pass
                pass
            if find_way:
                find_way = False
                break
                pass
            if error:
                error = False
                break
                pass
            pass
        pass
    print(shortest_way)
    pass


if __name__ == '__main__':
    main()
    pass
