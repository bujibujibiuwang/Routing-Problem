from collections import namedtuple, defaultdict
import re
import gurobipy as gp
from gurobipy import GRB
import math
import matplotlib.pyplot as plt

Customer = namedtuple("Customer", ['index', 'x', 'y', 'demand', 'ready', 'due', 'service'])


def length(point1, point2):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)


def read(path):
    with open(path, 'r') as file:
        data = file.read()
    lines = data.split('\n')
    parts = re.findall(r"\d+", lines[4])
    vehicle_count, vehicle_capacity = int(parts[0]), int(parts[1])

    points_list = []
    points_count = 0
    for k in range(9, len(lines)):
        line = lines[k]
        parts = re.findall(r"\d+", line)
        parts = [int(a) for a in parts]
        points_count += 1
        points_list.append(Customer._make(parts))

    return points_count, vehicle_count, vehicle_capacity, points_list


def MIP_model(points_count, vehicle_count, vehicle_capacity, points_list):
    """
    (1)已知数据
    """
    points = [i for i in range(points_count)]
    customers = [i for i in range(1, points_count)]
    edges = [(i, j) for i in range(points_count) for j in range(points_count) if i != j]
    distance = {(i, j): length(points_list[i], points_list[j]) for i, j in edges}
    vehicles = [*range(vehicle_count)]
    demand = [p.demand for p in points_list]
    time = [p.service for p in points_list]
    print(demand)
    print(time)
    BigM = 100000
    """
    (2)决策变量和目标函数
    """
    model = gp.Model('VRPTW')
    select = model.addVars(edges, vehicles, vtype=GRB.BINARY, name='x(i,j,k)')
    start = model.addVars(points, vehicles, vtype=GRB.CONTINUOUS, name='s(i,k)')
    model.setObjective(gp.quicksum(select[i, j, k] * distance[i, j] for k in vehicles for i, j in edges), GRB.MINIMIZE)
    """
    (3)约束条件
    """
    # (1)每个客户只访问一次
    model.addConstrs(select.sum(i, '*', '*') == 1 for i in customers)
    model.addConstrs(select.sum('*', j, '*') == 1 for j in customers)
    # (2)每辆车容量限制
    model.addConstrs(gp.quicksum(select[i, j, k] * demand[i] for i, j in edges) <= vehicle_capacity for k in vehicles)
    # (3)车辆起点约束
    model.addConstrs(select.sum(0, '*', k) == 1 for k in vehicles)
    # (4)车辆终点约束
    model.addConstrs(select.sum('*', 0, k) == 1 for k in vehicles)
    # (5)车辆到达客户节点后离开
    model.addConstrs(select.sum('*', j, k) == select.sum(j, '*', k) for j in customers for k in vehicles)
    # (6)车辆数量限制
    model.addConstr(select.sum(0, '*', '*') <= vehicle_count)
    model.addConstr(select.sum('*', 0, '*') <= vehicle_count)
    # (7)时间窗顺序约束
    model.addConstrs(start[i, k] + int(distance[(i, j)]) + time[i] - BigM * (1 - select[i, j, k])
                     <= start[j, k] for i, j in edges for k in vehicles)
    # (8)时间窗大小约束
    model.addConstrs(start[i, k] >= points_list[i].ready for i in customers for k in vehicles)
    model.addConstrs(start[i, k] <= points_list[i].due for i in customers for k in vehicles)

    model.addConstrs(start[i, k] <= BigM * select.sum(i, '*', k) for i in customers for k in vehicles)

    # model.setParam('TimeLimit', 10)
    model.optimize()

    arcs = []
    for k in vehicles:
        for i, j in edges:
            if select[i, j, k].x > 0.99:
                arcs.append((i, j))
    print(arcs)
    hash_table = defaultdict(list)
    for e in arcs:
        hash_table[e[0]].append(e[1])
    vehicle_tours = []
    for k in range(len(hash_table[0])):
        vehicle_tours.append([])
        vehicle_tours[k].append(0)
        vehicle_tours[k].append(hash_table[0][k])
        last = vehicle_tours[k][-1]
        while last != 0:
            vehicle_tours[k].append(hash_table[last][0])
            last = vehicle_tours[k][-1]
    return vehicle_tours


file_path = './R101.txt'
point_cnt, vehicle_cnt, vehicle_cap, points_ls = read(file_path)
print(point_cnt, vehicle_cnt, vehicle_cap, points_ls)
vehicle_cnt = 7
opt_tours = MIP_model(point_cnt, vehicle_cnt, vehicle_cap, points_ls)
print(opt_tours)


def draw(tours):
    for sub in tours:
        for i in range(len(sub)-1):
            plt.plot([points_ls[sub[i]].x, points_ls[sub[i+1]].x], [points_ls[sub[i]].y, points_ls[sub[i+1]].y], c='red')
            plt.scatter(points_ls[sub[i]].x, points_ls[sub[i]].y, c='black')
    plt.show()


draw(opt_tours)
