from dataclasses import dataclass
from typing import Optional, Dict
from collections import defaultdict
from InFileField import *
from datetime import datetime
import math
import pandas as pd



@dataclass
class Location:
    ID: str
    longitude: float
    latitude: float

class Vehicle:
    def __init__(self, id: str, origin: str, des: str, start: int, end: int,
                 max_distance: float, max_time: float, match: str, speed: float, max_load: int, unit_cost: float):
        self.id = id
        self.origin = origin
        self.des = des
        self.start = start
        self.end = end
        self.max_distance = max_distance
        self.max_time = max_time
        self.match = match
        self.speed = speed
        self.max_load = max_load
        self.unit_cost = unit_cost

        # 基于匹配关系得到车辆可以运输的order id
        self.alter_order_list = []
        # 基于匹配关系得到车辆可能经过的虚拟node id
        self.alter_node_list = []

    def update_alter_order(self, order_list: list):
        self.alter_order_list.extend(order_list)

    def update_alter_node(self, node: int):
        self.alter_node_list.append(node)



class Order:
    def __init__(self, id: str, match: str, quantity: int,
                 pick_loca: str, pick_service: float, pick_start: int, pick_end: int,
                 del_loca: str, del_service: float, del_start: int, del_end: int):
        self.id = id
        self.match = match
        self.quantity = quantity
        self.pick_loca = pick_loca
        self.pick_service = pick_service
        self.pick_start = pick_start
        self.pick_end = pick_end
        self.del_loca = del_loca
        self.del_service = del_service
        self.del_start = del_start
        self.del_end = del_end

class System:
    def __init__(self, path):
        self.path = path
        self.location_obj_dict: Dict[str, Location] = {}
        self.vehicle_obj_dict: Dict[str, Vehicle] = {}
        self.order_obj_dict: Dict[str, Order] = {}

        """
        基础信息
        """
        self.dist_matrix: Dict[tuple, float] = defaultdict(float)
        # loca是实际点，node是虚拟点
        self.dummy_node_dict: Dict[int, tuple] = {}
        self.dummy_node_count: int = 0
        self.base_datetime: Optional[datetime] = None
        """
        system初始化
        """
        self.create_environment()

    def create_environment(self):
        self.create_location()
        self.create_vehicle()
        self.create_order()
        self.get_dist_matrix()
        self.vehicle_map_order()

    def create_location(self):
        location_data = pd.read_excel(self.path, sheet_name=0)
        for row in location_data.itertuples():
            loca_name = getattr(row, InLocaFD.ID)
            location_obj = Location(ID=loca_name,
                                    longitude=getattr(row, InLocaFD.Longitude),
                                    latitude=getattr(row, InLocaFD.Latitude))
            self.location_obj_dict[loca_name] = location_obj

    def create_vehicle(self):
        vehicle_data = pd.read_excel(self.path, sheet_name=1)
        vehicle_data[InVehFD.EarliestStart] = pd.to_datetime(vehicle_data[InVehFD.EarliestStart])
        vehicle_data[InVehFD.LatestEnd] = pd.to_datetime(vehicle_data[InVehFD.LatestEnd])
        self.base_datetime = vehicle_data[InVehFD.EarliestStart].min()
        for index, row in vehicle_data.iterrows():
            veh_id = row[InVehFD.ID]
            match_rela = row[InVehFD.Match].split(',')
            start = self.time_diff_util(row[InVehFD.EarliestStart])
            end = self.time_diff_util(row[InVehFD.LatestEnd])
            vehicle_obj = Vehicle(id=veh_id, origin=row[InVehFD.Origin], des=row[InVehFD.Des], start=start, end=end,
                                  max_distance=row[InVehFD.MaxDist], max_time=row[InVehFD.MaxTime],
                                  match=match_rela, speed=row[InVehFD.Speed],
                                  max_load=row[InVehFD.Load], unit_cost=row[InVehFD.UnitCost])
            self.vehicle_obj_dict[veh_id] = vehicle_obj

    def create_order(self):
        order_data = pd.read_excel(self.path, sheet_name=2)
        for index, row in order_data.iterrows():
            order_id = row[InOrderFD.ID]
            match_rela = row[InVehFD.Match].split(',')
            pick_loca, del_loca = row[InOrderFD.PickLoca], row[InOrderFD.DelLoca]
            pick_start = self.time_diff_util(row[InOrderFD.PickStart])
            pick_end = self.time_diff_util(row[InOrderFD.PickEnd])

            del_start = self.time_diff_util(row[InOrderFD.DelStart])
            del_end = self.time_diff_util(row[InOrderFD.DelEnd])
            order_obj = Order(id=order_id, match=match_rela, quantity=row[InOrderFD.Quantity],
                              pick_loca=pick_loca, pick_service=row[InOrderFD.PickService],
                              pick_start=pick_start, pick_end=pick_end,
                              del_loca=del_loca, del_service=row[InOrderFD.DelService],
                              del_start=del_start, del_end=del_end)
            self.order_obj_dict[order_id] = order_obj

        count = 0
        for order_id, order_obj in self.order_obj_dict.items():
            count += 1
            self.dummy_node_dict[count] = (order_id, order_obj.pick_loca, order_obj.quantity)
            count += 1
            self.dummy_node_dict[count] = (order_id, order_obj.del_loca, -1 * order_obj.quantity)
        self.dummy_node_count = len(self.dummy_node_dict)

    def get_dist_matrix(self):
        def calc_dist(lng1, lat1, lng2, lat2):
            lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lng1, lat2, lng2])
            d_lat = lat2 - lat1
            d_lon = lon2 - lon1
            a = math.sin(d_lat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(d_lon / 2) ** 2
            c = 2 * math.asin(math.sqrt(a))
            r = 6371
            return c * r

        for loca_id_from, loca_obj_from in self.location_obj_dict.items():
            for loca_id_to, loca_obj_to in self.location_obj_dict.items():
                if loca_id_from != loca_id_to:
                    dist = calc_dist(loca_obj_from.longitude, loca_obj_from.latitude,
                                     loca_obj_to.longitude, loca_obj_to.latitude)
                    self.dist_matrix[(loca_id_from, loca_id_to)] = dist

    def vehicle_map_order(self):
        match_map_order = defaultdict(list)
        for order_id, order_obj in self.order_obj_dict.items():
            for match_mode in order_obj.match:
                match_map_order[match_mode].append(order_id)
        for veh_id, veh_obj in self.vehicle_obj_dict.items():
            for match_mode in veh_obj.match:
                veh_obj.update_alter_order(match_map_order[match_mode])
            for node_idx, node_info in self.dummy_node_dict.items():
                if node_info[0] in veh_obj.alter_order_list:
                    veh_obj.update_alter_node(node_idx)

    def time_diff_util(self, time: datetime):
        if time <= self.base_datetime:
            return 0
        else:
            return (time - self.base_datetime).seconds

    def print_system_info(self):
        for veh_id, veh_obj in self.vehicle_obj_dict.items():
            print(veh_obj.alter_order_list)
            print(veh_obj.alter_node_list)
        print(self.dummy_node_dict)
        for order_id, order_obj in self.order_obj_dict.items():
            print(order_obj.pick_start, order_obj.pick_end, order_obj.del_start, order_obj.del_end)
        print(self.dist_matrix)