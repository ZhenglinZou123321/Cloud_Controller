import os
import sumolib
import json
import pandas as pd
import traci



step = 0

# 车辆参数
MAX_SPEED = 11  # 最大速度 (m/s)
MIN_SPEED = 0
MAX_ACCEL = 3  # 最大加速度 (m/s^2)
MIN_ACCEL = -20


# 信号灯周期和时间参数
GREEN_TIME = 30  # 绿灯时间
RED_TIME = 30  # 红灯时间
CYCLE_TIME = GREEN_TIME + RED_TIME
dt = 0.2
N=40
L_safe = 4 + 3 #4米车长，3米间距
# 初始化交叉口车辆列表
vehicles = []

# IDM模型参数
V_0 = MAX_SPEED  # 期望速度 (m/s)，可根据实际情况调整
T = 1.5  # 期望车头时距 (s)，可根据实际情况调整
a_max = MAX_ACCEL  # 最大加速度 (m/s²)，与前面已定义的保持一致或按需调整
b = -1*MIN_ACCEL  # 舒适制动减速度 (m/s²)，可根据实际情况调整
s_0 = 2  # 最小间距 (m)，可根据实际情况调整
delta = 4  # 速度影响指数参数，可根据实际情况调整




# 获取当前脚本所在的目录
script_dir = os.path.dirname(os.path.abspath(__file__))
# 地图文件读取
net_file_path = os.path.join(script_dir, "Map_new.net.xml")
net = sumolib.net.readNet(net_file_path) 

# 图连接文件读取
with open("Graph/junction_index.json", "r") as f:
    junction_index_dict = json.load(f)

with open("Graph/lane_index.json", "r") as f:
    lane_index_dict = json.load(f)
    index_lane_dict = {v: k for k, v in lane_index_dict.items()}  # 构造反向字典

# 邻接矩阵读取
df = pd.read_csv("Graph/junction_adj_matrix.csv", index_col=0)
junc_adj_matrix = df.values

df = pd.read_csv("Graph/lane_adj_matrix.csv", index_col=0)
lane_adj_matrix = df.values

# 信号灯与所管辖的路口图
with open("Graph/traffic_light_info.json", "r") as f:
    data = json.load(f)
lane_to_traffic_light = data["lane_to_traffic_light"]
traffic_light_to_lanes = data["traffic_light_to_lanes"]
# 遍历字典中的每个键值对
for key, value in traffic_light_to_lanes.items():
    # 将列表转换为集合以去除重复元素
    traffic_light_to_lanes[key] = list(set(value))




# 线程字典
vehicle_threads = {}
junction_threads = {}

# 交通流量计数字典
previous_vehicle_edges = {}
junction_counts = {}

for key in traffic_light_to_lanes.keys():
    junction_counts[key] = 0


# 所要管控的路口id
Intelligent_Sigal_List = ['j5', 'j6', 'j7', 'j10','j11','j12']


# 数据存储结构
VehicleLib = {}
class Vehicle():
    def __init__(self,id,type):
        self.id = id #
        self.type = type # 'CAV' 'HDV'
        self.v = 0
        self.pos = (0.0,0.0)
        self.lane = '' 

    def update(self,v,pos,lane,leader):
        self.v = v
        self.pos = pos
        self.lane = lane
        self.leader = leader # getleader 的返回值 (leader_id,gap)

JuncLib = {}
class Junc():
    def __init__(self,id):
        self.id = id #
        self.vehicle_num = {}
        self.lane_ids = self.traffic_light_to_lanes[self.id]
        self.vehicle_num = {laneID:0 for laneID in self.lane_ids}


    def update(self):
        self.vehicle_num = {laneID:traci.lane.getLastStepVehicleNumber(laneID) for laneID in self.lane_ids}


LightLib = {}
class Light():
    def __init__(self,id):
        self.id = id #
        self.phase = '' #'r' 'g' 'y'
        self.remaining_time = 0
        self.nextphase = ''
    def update(self,phase,remaining_time):
        self.phase = phase
        self.remaining_time = remaining_time
        if self.phase == 'r':
            self.nextphase = 'g'
        elif self.phase == 'g':
            self.nextphase = 'y'
        else:
            self.nextphase = 'r'
    