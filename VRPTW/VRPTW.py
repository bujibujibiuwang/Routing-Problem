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
    demand = [p.demand for p in points_list]
    time = [p.service for p in points_list]
    BigM = 100000
    """
    (2)决策变量和目标函数
    """
    model = gp.Model('VRP with time window')
    select = model.addVars(edges, vtype=GRB.BINARY, name='select')
    flow = model.addVars(points, lb=0, ub=vehicle_capacity, vtype=GRB.CONTINUOUS, name='flow')
    flow[0].UB = 0
    start = model.addVars(points, vtype=GRB.CONTINUOUS, name='start')
    model.setObjective(select.prod(distance), GRB.MINIMIZE)
    """
    (3)约束条件
    """
    # (1) 每个客户点一条出边和入边
    model.addConstrs(select.sum(i, '*') == 1 for i in customers)
    model.addConstrs(select.sum('*', j) == 1 for j in customers)

    # (2) 车辆数量约束
    model.addConstr(select.sum(0, '*') <= vehicle_count)
    model.addConstr(select.sum('*', 0) <= vehicle_count)

    # (3) 流量约束
    # model.addConstrs(flow[i] + demand[i] <= flow[j] + vehicle_capacity*(1 - select[i, j]) for i, j in edges if j != 0)
    # model.addConstrs(flow[i] + demand[i] >= flow[j] - BigM * (1 - select[i, j]) for i, j in edges if j != 0)

    model.addConstrs((select[i, j] == 1) >> (flow[i] + points_list[j].demand == flow[j])
                     for i, j in edges if j != 0)
    model.addConstrs(points_list[i].demand <= flow[i] for i in customers)
    model.addConstrs(flow[i] <= vehicle_capacity for i in customers)

    # (4)时间约束
    model.addConstrs(start[i] + int(distance[(i, j)]) + time[i] - BigM * (1 - select[i, j])
                     <= start[j] for i, j in edges if j != 0)
    model.addConstrs(start[i] >= points_list[i].ready for i in points)
    model.addConstrs(start[i] <= points_list[i].due for i in points)
    """
    (4)求解模型
    """
    model.optimize()
    arcs = [e for e in edges if select[e].x > 0.99]
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


file_path = './C101.txt'
point_cnt, vehicle_cnt, vehicle_cap, points_ls = read(file_path)
opt_tours = MIP_model(point_cnt, vehicle_cnt, vehicle_cap, points_ls)
for i, t in enumerate(opt_tours):
    print(f'route{i}:{t}', end='\n')


def draw(tours):
    for sub in tours:
        for i in range(len(sub)-1):
            plt.plot([points_ls[sub[i]].x, points_ls[sub[i+1]].x], [points_ls[sub[i]].y, points_ls[sub[i+1]].y], c='red')
            plt.scatter(points_ls[sub[i]].x, points_ls[sub[i]].y, c='black')
    plt.show()


draw(opt_tours)
