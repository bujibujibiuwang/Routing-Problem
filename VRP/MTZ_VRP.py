from collections import namedtuple, defaultdict
import gurobipy as gp
from gurobipy import GRB
import math
import matplotlib.pyplot as plt


Customer = namedtuple("Customer", ['index', 'demand', 'x', 'y'])


def length(point1, point2):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)


def read(path):
    with open(path, 'r') as file:
        data = file.read()
    lines = data.split('\n')
    parts = lines[0].split(' ')
    points_count, vehicle_count, vehicle_capacity = int(parts[0]), int(parts[1]), int(parts[2])
    points_list = []
    for i in range(1, points_count + 1):
        line = lines[i]
        parts = line.split()
        points_list.append(Customer(i - 1, int(parts[0]), float(parts[1]), float(parts[2])))
    return points_count, vehicle_count, vehicle_capacity, points_list


def MIP_model(points_count, vehicle_count, vehicle_capacity, points_list):
    """
    (1)已知数据
    """
    points = [i for i in range(points_count)]
    customers = [i for i in range(1, points_count)]
    edges = [(i, j) for i in range(points_count) for j in range(points_count) if i != j]
    distance = {(i, j): length(points_list[i], points_list[j]) for i, j in edges}
    """
    (2)决策变量和目标函数
    """
    model = gp.Model('VRP')
    select = model.addVars(edges, vtype=GRB.BINARY, name='select')
    flow = model.addVars(customers, vtype=GRB.CONTINUOUS, name='flow')
    model.setObjective(select.prod(distance), GRB.MINIMIZE)
    """
    (3)约束条件
    """
    # 每个客户点一条出边
    model.addConstrs(gp.quicksum(select[i, j] for j in points if j != i) == 1 for i in customers)
    # 每个客户点一条入边
    model.addConstrs(gp.quicksum(select[i, j] for i in points if i != j) == 1 for j in customers)
    # 流量顺序约束
    model.addConstrs((select[i, j] == 1) >> (flow[i] + points_list[j].demand == flow[j])
                     for i, j in edges if i != 0 and j != 0)
    # 流量大小约束
    model.addConstrs(points_list[i].demand <= flow[i] for i in customers)
    model.addConstrs(flow[i] <= vehicle_capacity for i in customers)
    # 车辆数量约束
    model.addConstr(gp.quicksum(select[0, j] for j in customers) <= vehicle_count)
    model.addConstr(gp.quicksum(select[i, 0] for i in customers) <= vehicle_count)
    """
    (4)求解模型
    """
    model.optimize()
    arcs = [e for e in edges if select[e].x > 0.99]
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


path = './vrp_16_3_1'
points_cnt, vehicle_cnt, vehicle_cap, points_ls = read(path)
opt_tours = MIP_model(points_cnt, vehicle_cnt, vehicle_cap, points_ls)


def draw(tours):
    for sub in tours:
        for i in range(len(sub)-1):
            plt.plot([points_ls[sub[i]].x, points_ls[sub[i+1]].x], [points_ls[sub[i]].y, points_ls[sub[i+1]].y], c='red')
            plt.scatter(points_ls[sub[i]].x, points_ls[sub[i]].y, c='black')
    plt.show()


draw(opt_tours)