from pulp import *
import networkx as nx
import matplotlib.pyplot as plt
from PDPTWSystem import System



class Solver:
    def __init__(self, system: System):
        self.system = system
        self.x_vars: Dict = {}
        self.a_vars: Dict = {}
        self.q_vars: Dict = {}
        self.w_vars: Dict = {}
        self.objs: float = 0

    def run(self):
        self.create_model()
        self.add_vars()
        self.add_objs()
        self.add_cons()
        self.solve_model()

    def add_vars(self):
        self.add_edge_var()
        self.add_node_var()

    def add_objs(self):
        self.add_trans_cost_objs()

    def add_cons(self):
        self.add_basic_cons()
        self.add_load_cons()
        self.add_time_cons()
        self.add_seq_cons()
        self.add_limit_cons()

    def create_model(self):
        self.model = LpProblem(name=f'PickDeliveryProblem', sense=LpMinimize)

    def add_edge_var(self):
        # 变量x_kij: 车辆k是否经过边ij，ij是带order维度的虚拟扩展点，以及车辆的起始点
        for veh_k, veh_obj in self.system.vehicle_obj_dict.items():
            for node_i in veh_obj.alter_node_list:
                for node_j in veh_obj.alter_node_list:
                    if node_i != node_j:
                        self.x_vars[(veh_k, node_i, node_j)] = LpVariable(name=f'x_{veh_k}_{node_i}_{node_j}',
                                                                          lowBound=0, upBound=1, cat=LpBinary)
        for veh_k, veh_obj in self.system.vehicle_obj_dict.items():
            node_origin, node_des = 0, self.system.dummy_node_count + 1
            for node in veh_obj.alter_node_list:
                    self.x_vars[(veh_k, node_origin, node)] = LpVariable(name=f'{veh_k}_{node_origin}_{node}',
                                                                         lowBound=0, upBound=1, cat=LpBinary)
                    self.x_vars[(veh_k, node, node_des)] = LpVariable(name=f'{veh_k}_{node}_{node_des}',
                                                                      lowBound=0, upBound=1, cat=LpBinary)

    def add_node_var(self):
        # 变量q_ki：车辆k到达点i的载货量
        # 变量a_ki：车辆k到达点i的时间
        # 变量w_ki：车辆k在点i的等待时间
        for veh_k, veh_obj in self.system.vehicle_obj_dict.items():
            for node_i in veh_obj.alter_node_list:
                self.q_vars[(veh_k, node_i)] = LpVariable(name=f'q_{veh_k}_{node_i}', lowBound=0, cat=LpContinuous)
                self.a_vars[(veh_k, node_i)] = LpVariable(name=f'a_{veh_k}_{node_i}', lowBound=0, cat=LpContinuous)
                self.w_vars[(veh_k, node_i)] = LpVariable(name=f'w_{veh_k}_{node_i}', lowBound=0, cat=LpContinuous)
            node_origin, node_des = 0, self.system.dummy_node_count + 1
            self.q_vars[(veh_k, node_origin)] = LpVariable(name=f'q_{veh_k}_{node_origin}', lowBound=0, cat=LpContinuous)
            self.a_vars[(veh_k, node_origin)] = LpVariable(name=f'a_{veh_k}_{node_origin}', lowBound=0, cat=LpContinuous)
            self.w_vars[(veh_k, node_origin)] = LpVariable(name=f'w_{veh_k}_{node_origin}', lowBound=0, cat=LpContinuous)

            self.q_vars[(veh_k, node_des)] = LpVariable(name=f'q_{veh_k}_{node_des}', lowBound=0, cat=LpContinuous)
            self.a_vars[(veh_k, node_des)] = LpVariable(name=f'a_{veh_k}_{node_des}', lowBound=0, cat=LpContinuous)
            self.w_vars[(veh_k, node_des)] = LpVariable(name=f'w_{veh_k}_{node_des}', lowBound=0, cat=LpContinuous)


    def add_trans_cost_objs(self):
        # 对于变量kij，找到ij的距离和k的单位运输成本，即可计算总运输成本
        for key in self.x_vars.keys():
            k, i, j = key[0], key[1], key[2]
            veh_obj = self.system.vehicle_obj_dict[k]
            if i == 0:
                loca_i_name = veh_obj.origin
            else:
                loca_i_name = self.system.dummy_node_dict[i][1]
            if j == len(self.system.dummy_node_dict)+1:
                loca_j_name = veh_obj.des
            else:
                loca_j_name = self.system.dummy_node_dict[j][1]
            if loca_i_name != loca_j_name:
                dist = self.system.dist_matrix[(loca_i_name, loca_j_name)]
                self.objs += self.x_vars[key] * veh_obj.unit_cost * dist
        self.model += (self.objs, f'minimize_transport_cost')

    def add_basic_cons(self):
        # (1) 保证每个货物都被配送：对所有虚拟点中的每个pick点，要求必须存在(pick,j)
        for pick_node in self.system.dummy_node_dict.keys():
           if pick_node % 2 != 0:
               lhs_origin = 0
               for key in self.x_vars.keys():
                   k, i, j = key[0], key[1], key[2]
                   if i == pick_node:
                        lhs_origin += self.x_vars[key]
               self.model += (lhs_origin == 1, f'{pick_node}_pick_cons')

        # (3) 保证取货后要有对应的送货：车辆k经过pick点也一定也要经过对应的del点
        for veh_k, veh_obj in self.system.vehicle_obj_dict.items():
            for node in veh_obj.alter_node_list:
                if node % 2 != 0:
                    pick_node, del_node = node, node + 1
                    lhs_origin, rhs = 0, 0
                    for key in self.x_vars.keys():
                        k, i, j = key[0], key[1], key[2]
                        if k == veh_k and i == pick_node:
                            lhs_origin += self.x_vars[key]
                        if k == veh_k and i == del_node:
                            rhs += self.x_vars[key]
                    self.model += (lhs_origin == rhs, f'{veh_k}_{pick_node}_{del_node}_pair_cons')

        # (4) 路径平衡约束：每辆车一定经过从起点出发和回到终点，其他虚拟点要保证有进有出
        for veh_k, veh_obj in self.system.vehicle_obj_dict.items():
            lhs_origin, lhs_des = 0, 0
            for node in veh_obj.alter_node_list:
                lsh, rhs = 0, 0
                for key in self.x_vars.keys():
                    k, i, j = key[0], key[1], key[2]
                    if k == veh_k and j == node and i == 0:
                      lhs_origin += self.x_vars[key]
                    if k == veh_k and i == node and j == (self.system.dummy_node_count + 1):
                        lhs_des += self.x_vars[key]
                    if k == veh_k and i == node:
                        lsh += self.x_vars[key]
                    if k == veh_k and j == node:
                        rhs += self.x_vars[key]
                self.model += (lsh == rhs, f'{veh_k}_{node}_inout_cons')
            self.model += (lhs_origin == 1, f'{veh_k}_origin_cons')
            self.model += (lhs_des == 1, f'{veh_k}_des_cons')


    def add_load_cons(self):
        # (5) 负载平衡约束：如果车辆k经过边(i,j)，那么到达i的负载量+节点j的负载量=到达j的负载量
        for key in self.x_vars.keys():
            k, i, j = key[0], key[1], key[2]
            veh_obj = self.system.vehicle_obj_dict[k]
            big_M = 2 * veh_obj.max_load
            lhs = self.q_vars[(k, j)]
            if i == 0:
                rhs = self.q_vars[(k, i)] + (self.x_vars[key] - 1) * big_M
            else:
                load_i = self.system.dummy_node_dict[i][2]
                rhs = self.q_vars[(k, i)] + load_i + (self.x_vars[key] - 1) * big_M
            # 为了将约束线性化，引入bigM，必须要用不等式，导致该约束限制不了环路的出现
            self.model += (lhs >= rhs, f'{key}_load_balance_cons')

        # (6) 负载最大约束：车辆k到达节点i的负载量不能大于最大载量
        for key in self.q_vars.keys():
            k, i = key[0], key[1]
            veh_obj = self.system.vehicle_obj_dict[k]
            self.model += (self.q_vars[key] <= veh_obj.max_load, f'{k}_{i}_max_load_cons')

    def add_time_cons(self):
        # (7) 时间平衡约束：如果车辆k经过ij，那么到达j的时间=到达i的时间+在i等待的时间+服务i的时间+运输耗时
        # 可以限制环路
        for key in self.x_vars.keys():
            k, i, j = key[0], key[1], key[2]
            veh_obj = self.system.vehicle_obj_dict[k]
            big_M = 2 * veh_obj.end
            if j == self.system.dummy_node_count + 1:
                loca_j_name = veh_obj.des
            else:
                loca_j_name = self.system.dummy_node_dict[j][1]
            if i == 0:
                loca_i_name = veh_obj.origin
                service_i = 0
            else:
                loca_i_name = self.system.dummy_node_dict[i][1]
                order_id = self.system.dummy_node_dict[i][0]
                order_obj = self.system.order_obj_dict[order_id]
                service_i = order_obj.pick_service
            trans_time = round((self.system.dist_matrix[(loca_i_name, loca_j_name)] / veh_obj.speed) * 3600)
            lhs = self.a_vars[(k, i)] + self.w_vars[(k, i)] + trans_time + (self.x_vars[key] - 1) * big_M + service_i
            self.model += (lhs <= self.a_vars[(k, j)], f'{k}_{i}_{j}_time_balance_cons')

        # (8)(9)(10) 时间窗约束
        # 车辆k在节点i的等待时间=max(0, 节点i可开始服务的时间-到达节点i的时间）
        # 节点i最早开始服务 <= 在节点i的等待时间 + 到达节点i的时间 <= 节点i最晚开始服务
        # 起点的到达时间>=车辆的最早开始时间，终点的到达时间<=车辆的最晚时间
        # 约束2可以实现约束13，去掉约束1也能避免线性化操作
        for key in self.a_vars.keys():
            k, i = key[0], key[1]
            veh_obj = self.system.vehicle_obj_dict[k]
            lhs = self.a_vars[key] + self.w_vars[key]
            if i == 0 or i == self.system.dummy_node_count + 1:
                self.model += (lhs <= veh_obj.end, f'{k}_{i}_window_end_cons')
                self.model += (lhs >= veh_obj.start, f'{k}_{i}_window_start_cons')
            else:
                order_id = self.system.dummy_node_dict[i][0]
                order_obj = self.system.order_obj_dict[order_id]
                if i % 2 != 0:
                    self.model += (lhs <= order_obj.pick_end, f'{k}_{i}_window_end_cons')
                    self.model += (lhs >= order_obj.pick_start, f'{k}_{i}_window_start_cons')
                else:
                    self.model += (lhs <= order_obj.del_end, f'{k}_{i}_window_end_cons')
                    self.model += (lhs >= order_obj.del_start, f'{k}_{i}_window_start_cons')

    def add_limit_cons(self):
        # (11) 最大运输距离限制：车辆k经过的ij总运输距离不能超过车辆最大距离
        for veh_k, veh_obj in self.system.vehicle_obj_dict.items():
            lhs = 0
            for key in self.x_vars.keys():
                k, i, j = key[0], key[1], key[2]
                if k == veh_k:
                    if j == self.system.dummy_node_count + 1:
                        loca_j_name = veh_obj.des
                    else:
                        loca_j_name = self.system.dummy_node_dict[j][1]
                    if i == 0:
                        loca_i_name = veh_obj.origin
                    else:
                        loca_i_name = self.system.dummy_node_dict[i][1]
                    lhs += self.x_vars[key] * self.system.dist_matrix[(loca_i_name, loca_j_name)]
            self.model += (lhs <= veh_obj.max_distance, f'{veh_k}_distance_limit_cons')

        # (12) 最大运输时间限制：车辆k到达终点的时间-车辆从起点出发的时间不能超过最大时长
        for veh_k, veh_obj in self.system.vehicle_obj_dict.items():
            time_delta = self.a_vars[(veh_k, self.system.dummy_node_count + 1)] - self.a_vars[(veh_k, 0)]
            self.model += (time_delta <= (veh_obj.max_time * 3600), f'{veh_k}_time_limit_cons')

    def add_seq_cons(self):
        # (13) 保证货物先取后送
        for key in self.a_vars.keys():
            k, i = key[0], key[1]
            veh_obj = self.system.vehicle_obj_dict[k]
            if i % 2 != 0 and i != 0 and i != (self.system.dummy_node_count + 1):
                node_pick, node_del = i, i + 1
                order_id, loca_i_name, _ = self.system.dummy_node_dict[node_pick]
                loca_j_name = self.system.dummy_node_dict[node_del][1]
                order_obj = self.system.order_obj_dict[order_id]
                trans_time = round((self.system.dist_matrix[(loca_i_name, loca_j_name)] / veh_obj.speed) * 3600)
                lhs = self.a_vars[(k, node_pick)] + self.w_vars[(k, node_pick)] + order_obj.pick_service + trans_time
                self.model += (lhs <= self.a_vars[(k, node_del)], f'{k}_{i}_{i+1}_seq_cons')

    def solve_model(self):
        self.model.writeLP('PickDeliveryProblem.lp')
        self.model.solve(GUROBI(msg=True))
        print(LpStatus[self.model.status])
        if LpStatus[self.model.status] == 'Infeasible':
            for cons in self.model.constraints.values():
                print(f'{cons.name}: {cons.value()}')


class Result:
    def __init__(self, system: System, solver: Solver):
        self.system = system
        self.solver = solver

    def build_graph(self):
        graph = nx.DiGraph()
        graph.add_nodes_from(self.system.dummy_node_dict.keys())
        graph.add_node(0)
        graph.add_node(self.system.dummy_node_count + 1)
        for (veh_k, node_i, node_j), var in self.solver.x_vars.items():
            if var and var.varValue > 1e-5:
                print((veh_k, node_i, node_j))
                graph.add_edge(node_i, node_j)
        print(f'-----------------q-----------------')
        for (veh_k, node_i), var in self.solver.q_vars.items():
            print(veh_k, node_i, var.varValue)
        print(f'-----------------a-----------------')
        for (veh_k, node_i), var in self.solver.a_vars.items():
            print(veh_k, node_i, var.varValue)
        print(graph.edges)
        nx.draw(graph, with_labels=True)
        plt.show()



path = 'PDPTWData.xls'
system = System(path)
system.print_system_info()
solver = Solver(system)
result = Result(system, solver)
solver.run()
result.build_graph()