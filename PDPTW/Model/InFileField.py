from enum import Enum

class InLocaFD:
    ID = '停靠点ID'
    Longitude = '经度'
    Latitude = '纬度'

class InVehFD:
    ID = '车辆ID'
    Origin = '起始停靠点'
    Des = '结束停靠点'
    EarliestStart = '最早开始时间'
    LatestEnd = '最晚结束时间'
    MaxDist = '最大行驶距离(千米)'
    MaxTime = '最大行驶时间(小时)'
    Match = '匹配关系'
    Speed = '速度km/h'
    Load = '载量'
    UnitCost = '单位距离成本'

class InOrderFD:
    ID = '货物ID'
    Match = '匹配关系'
    Quantity = '货物量'
    PickLoca = '提货地点'
    PickService = '提货服务时间(m)'
    PickStart = '提货时间窗(开始)'
    PickEnd = '提货时间窗(结束)'
    DelLoca = '送货地点'
    DelService = '送货服务时间(m)'
    DelStart = '送货时间窗(开始)'
    DelEnd = '送货时间窗(结束)'


class SolveConfig:
    Heuristic = 'NoRelHeurTime'


class PDtype(Enum):
    pick = '取货'
    delivery = '送货'
    origin = '起点'
    des = '终点'
