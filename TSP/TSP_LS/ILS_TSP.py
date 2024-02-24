"""
ILS for TSP
author：carrot
time：10/1/2023
"""
import numpy as np
import random
import matplotlib.pyplot as plt
from collections import namedtuple
from Local2opt import Local2opt

Point = namedtuple('Point', ['index', 'x', 'y'])


def read(path):
    with open(path, 'r') as file:
        data = file.read()
    lines = data.split('\n')
    node_count = int(lines[0])
    points = []
    for i in range(1, node_count + 1):
        line = lines[i]
        part = line.split()
        points.append(Point(int(part[0]), float(part[1]), float(part[2])))
    return node_count, points


def generate_initial_solution(node_count):
    return np.random.permutation(node_count)


def perturb(path):
    index = len(path) // 4
    part_one = 1 + random.randint(0, index)
    part_two = part_one + 1 + random.randint(0, index)
    part_three = part_two + 1 + random.randint(0, index)
    return path[:part_one] + path[part_three:] + path[part_two:part_three] + path[part_one:part_two]


def iterated_local_search(node_count, points, max_search):
    init_path = generate_initial_solution(node_count)
    opt = Local2opt(points, init_path)
    best_path, best_dist = opt.two_opt()
    for i in range(max_search):
        current_path = perturb(best_path)
        opt = Local2opt(points, current_path)
        new_path, new_dist = opt.two_opt()
        if new_dist < best_dist:
            best_path = new_path
            best_dist = new_dist
        if i % 50 == 0:
            print(f'number {i}, best:{best_dist}, new:{new_dist}')
    return best_path, best_dist


def plot(path, points):
    for i in range(len(path)-1):
        node1 = path[i]
        node2 = path[i + 1]
        plt.plot([points[node1].x, points[node2].x], [points[node1].y, points[node2].y], c='r')
        plt.scatter(points[node1].x, points[node1].y, c='black')
    plt.plot([points[path[-1]].x, points[path[0]].x], [points[path[-1]].y, points[path[0]].y], c='r')
    plt.scatter(points[path[-1]].x, points[path[-1]].y, c='black')
    plt.show()


node_count, points = read('../TSP_Gurobi/Berlin52.txt')
p, d = iterated_local_search(node_count, points, 700)
plot(p, points)







