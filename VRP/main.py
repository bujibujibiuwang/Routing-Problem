from collections import namedtuple


Customer = namedtuple("Customer", ['index', 'demand', 'x', 'y'])


def read(path):
    with open(path, 'r') as file:
        data = file.read()
    parts = data[0].split(' ')
    points_count, vehicle_count, vehicle_capacity = int(parts[0]), int(parts[1]), int(parts[2])
    customers = []
    for i in range(1, points_count + 1):
        line = data[i]
        parts = line.split()
        customers.append(Customer(i - 1, int(parts[0]), float(parts[1]), float(parts[2])))
    depot = customers[0]
    return points_count, vehicle_count, vehicle_capacity, customers, depot


def MIP_model(points_count, vehicle_count, vehicle_capacity, customers, depot):
    points = [i for i in range(points_count)]
    customers = [i for i in range(1, points_count)]




