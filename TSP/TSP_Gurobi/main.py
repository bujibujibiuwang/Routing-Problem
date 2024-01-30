"""
Gurobi for TSP
author：carrot
time：2024/1/30
"""
import gurobipy as gp
from collections import namedtuple
import math
from itertools import combinations
from gurobipy import GRB
import matplotlib.pyplot as plt

'''
(1)数据处理
'''
Point = namedtuple('Point', ['index', 'x', 'y'])


def read(path):
    with open(path, 'r') as file:
        data = file.read()
    lines = data.split('\n')
    node_count = int(lines[0])
    points_list = []
    for i in range(1, node_count + 1):
        line = lines[i]
        part = line.split()
        points_list.append(Point(int(part[0]), float(part[1]), float(part[2])))
    return node_count, points_list


def distance(point1, point2):
    diff_x = point1.x - point2.x
    diff_y = point1.y - point2.y
    diff = math.sqrt(diff_x ** 2 + diff_y ** 2)
    return diff


number, points = read('./berlin52.txt')
dist = {(p1.index, p2.index): distance(p1, p2) for p1, p2 in combinations(points, 2)}
print(dist)
'''
(2)构建模型
'''
m = gp.Model()
# 决策变量
decision_var = m.addVars(dist.keys(), obj=dist, vtype=GRB.BINARY, name='x')
# 对称约束
for i, j in decision_var.keys():
    decision_var[j, i] = decision_var[i, j]
# 进出约束
m.addConstrs(decision_var.sum(p.index, '*') == 2 for p in points)


# 子圈消除约束：callback + lazy constraints
def sub_tour_eliminate(model, where):
    if where == GRB.Callback.MIPSOL:
        print("sub")
        values = model.cbGetSolution(model._vars)
        selected = gp.tuplelist((i, j) for i, j in model._vars.keys() if values[i, j] > 0.5)
        cycle = get_sub_tour(selected)
        if len(cycle) < number:
            model.cbLazy(gp.quicksum(model._vars[i, j] for i, j in combinations(cycle, 2)) <= len(cycle) - 1)


# 找到最小的子圈，用于添加lazy constraints
def get_sub_tour(edges):
    unvisited = [p.index for p in points]
    cycle = edges[:]
    while unvisited:
        current_cycle = []
        neighbors = unvisited
        while neighbors:
            current = neighbors[0]
            current_cycle.append(current)
            unvisited.remove(current)
            neighbors = [j for i, j in edges.select(current, '*') if j in unvisited]
        if len(current_cycle) < len(cycle):
            cycle = current_cycle
    return cycle


'''
(4)求解模型
'''
m._vars = decision_var
m.Params.lazyConstraints = 1
m.optimize(sub_tour_eliminate)
'''
(5)结果展示
'''
vals = m.getAttr('x', decision_var)
select_opt = gp.tuplelist((i, j) for i, j in vals.keys() if vals[i, j] > 0.5)
tour = get_sub_tour(select_opt)
assert len(tour) == number


def plot(ans, city):
    for node1, node2 in ans:
        plt.plot([city[node1-1].x, city[node2-1].x], [city[node1-1].y, city[node2-1].y], c='r')
        plt.scatter(city[node1-1].x, city[node1-1].y, c='black')
    plt.show()


plot(select_opt, points)

