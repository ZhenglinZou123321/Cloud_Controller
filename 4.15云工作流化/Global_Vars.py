import os
import time
import sumolib
import json
import pandas as pd
import traci
import traci.constants
import numpy as np



class Sim_info():
    def __init__(self):
        self.now_time = traci.simulation.getTime() #simulation time in s
        self.step = 0.2

    def update(self):
        self.now_time += self.step

    def Conv_from_dict(self,dict):
        self.now_time = dict['now_time']
        self.step = dict['step']

simulate_info = Sim_info()

def get_remaining_phase_and_time(lane_id): #获取信号灯当前相位和剩余时间
    # 按照固定字符进行分割
    x, rest = lane_id.split("t", 1)  # 分割出 X 和剩余部分
    intersection_id, z = rest.split("_", 1)  # 分割出 Y 和 Z
    # 获取当前仿真时间
    current_time = simulate_info.now_time
    # 获取下一个信号切换的时间
    next_switch_time = traci.trafficlight.getNextSwitch(intersection_id)
    # 计算剩余时间 秒
    remaining_time = next_switch_time - current_time
    current_phase = traci.trafficlight.getRedYellowGreenState(intersection_id)[traci.trafficlight.getControlledLanes(intersection_id).index(lane_id)]
    return current_phase.lower(),max(remaining_time, 0)  # 防止负值





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
b = a_max  # 舒适制动减速度 (m/s²)，可根据实际情况调整
s_0 = 1  # 最小间距 (m)，可根据实际情况调整
delta = 6  # 速度影响指数参数，可根据实际情况调整  delta较大，车辆会更早的去往期望速度变化




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
reversed_lane_dict = {str(v): k for k, v in lane_index_dict.items()}

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
Lanes_length = {laneID:traci.lane.getLength(laneID) for laneID in lane_index_dict.keys()}



VehicleLib = {}
class Vehicle():
    def __init__(self,id,type):
        self.id = id #
        self.type = type # 'CAV' 'HDV'
        # 订阅车辆数据
        traci.vehicle.subscribe(self.id, [
            traci.constants.VAR_SPEED,
            traci.constants.VAR_LANE_ID,
            traci.constants.VAR_LANEPOSITION,
            traci.constants.VAR_ROAD_ID
        ])
        self.length = traci.vehicle.getLength(self.id)
        self.running = True
        traci.vehicle.subscribeLeader(self.id)
        self.control_signal = []
        self.control_signal_new = []
        self.time_when_cal = 0
        self.dt = dt
        self.time_entry = simulate_info.now_time
        self.time_exit = 0
        self.update()

    def update(self):
        try:
            # 批量获取订阅的车辆数据
            subscription_results = traci.vehicle.getSubscriptionResults(self.id)
            if subscription_results is None:
                self.running = False
                return
            self.speed = subscription_results[traci.constants.VAR_SPEED]
            self.lane = subscription_results[traci.constants.VAR_LANE_ID]
            self.leader = subscription_results[traci.constants.VAR_LEADER]# 元组(vehicle_id,idstance)
            self.laneposition = subscription_results[traci.constants.VAR_LANEPOSITION]
            self.RoadID = subscription_results[traci.constants.VAR_ROAD_ID]
        except:
            self.running = False

    def get_control_signal(self,time_now, dt):
        if int((time_now-self.time_when_cal)/dt) <= len(self.control_signal)-1:
            return self.control_signal[int((time_now-self.time_when_cal)/dt)]
        else:
            return None
    def idm_acceleration(self, current_speed, front_vehicle_speed, gap,  front_vehicle_id=None):
        """
        根据IDM模型计算车辆的加速度。

        参数:
        current_speed (float): 当前车辆速度 (m/s)
        front_vehicle_speed (float): 前车速度 (m/s)
        gap (float): 当前车与前车的间距 (m)
        front_vehicle_id (str, 可选): 前车的ID，用于调试或其他可能的拓展需求，默认为None

        返回:
        float: 根据IDM模型计算出的当前车辆加速度 (m/s²)
        """
        
        relative_speed = current_speed - front_vehicle_speed
        s_star = s_0 + current_speed * T + (current_speed * relative_speed) / (2 * np.sqrt(a_max * b))
        acceleration = a_max * (1 - (current_speed / V_0) ** delta - (s_star / gap) ** 2)

        return min(max(acceleration,MIN_ACCEL), MAX_ACCEL)
    
    def acceleration_control(self,time_now,dt,junc_id,lane_id):
        mpc_acc = self.get_control_signal(time_now, dt)
        if self.leader != None and (VehicleLib[self.leader[0]].lane == self.lane): #如果它有前车，并且前车在同一车道上
            idm_acc = self.idm_acceleration(self.speed, VehicleLib[self.leader[0]].speed, self.leader[1], front_vehicle_id=None)
        else: #如果它没有前车，或前车与其不在一个车道那么就是头车，需要考虑红绿灯约束
            light = LightLib[junc_id]
            print(light.phase[lane_id])
            if light.phase[lane_id] in ['r','y']:
                #红绿灯约束
                relative_speed = self.speed - 0
                s_star = s_0 + self.speed * T + (self.speed * relative_speed) / (2 * np.sqrt(-1*MIN_ACCEL * b)) #b=3 Min_accel = -20
                gap = JuncLib[junc_id].lanes_length[lane_id] - 1 - self.laneposition
                idm_acc = -1*MIN_ACCEL * (1 - (self.speed / V_0) ** delta - (s_star / gap) ** 2)
            else:
                idm_acc = 99
            
        if mpc_acc != None and idm_acc != None:
            acc = min(mpc_acc,idm_acc)
            print(f"{self.id} MPC:{mpc_acc} IDM:{idm_acc}")
            if acc == mpc_acc:
                traci.vehicle.setColor(self.id, (0, 0, 255)) # MPC = blue
            else:
                traci.vehicle.setColor(self.id, (255, 0, 0)) # IDM = red

            traci.vehicle.setSpeedMode(self.id, 0000000)  # 关闭跟驰模型
            traci.vehicle.setAcceleration(self.id, acc, dt)
            print(f"{self.id} 速度施加成功")
        else:
            traci.vehicle.setSpeedMode(self.id, 31)
            print(f"{self.id} 速度施加失败")
    def Conv_from_dict(self, dict):
        self.speed = dict['speed']
        self.lane = dict['lane']
        self.leader = dict['leader']
        self.laneposition = dict['laneposition']
        self.RoadID = dict['RoadID']
    def Conv_to_dict(self):
        return {
            'speed': self.speed,
            'lane': self.lane,
            'leader': self.leader,
            'laneposition': self.laneposition,
            'RoadID': self.RoadID
        }


JuncLib = {}
Vehicle_IDs = set()
class Junc():
    def __init__(self,id):
        self.id = id #
        self.vehicle_num = {}
        self.lane_ids = traffic_light_to_lanes[self.id]
        self.vehicle_num = {laneID:0 for laneID in self.lane_ids}
        self.lanes_length = {laneID:traci.lane.getLength(laneID) for laneID in self.lane_ids}
        self.vehicle_ids = {laneID:[] for laneID in self.lane_ids}
        # 订阅车道数据
        for lane_id in self.lane_ids:
            traci.lane.subscribe(lane_id, [traci.constants.LAST_STEP_VEHICLE_NUMBER, traci.constants.LAST_STEP_VEHICLE_ID_LIST])
        self.update()


    def update(self):
        for lane_id in self.lane_ids:
            subscription_results = traci.lane.getSubscriptionResults(lane_id)
            self.vehicle_num[lane_id] = subscription_results[traci.constants.LAST_STEP_VEHICLE_NUMBER]
            self.vehicle_ids[lane_id] = subscription_results[traci.constants.LAST_STEP_VEHICLE_ID_LIST]
            Vehicle_IDs.update(subscription_results[traci.constants.LAST_STEP_VEHICLE_ID_LIST])
            for vehicle_id in Vehicle_IDs:
                if vehicle_id not in VehicleLib:
                    vehicleclass = Vehicle(vehicle_id,vehicle_id[0:3])
                    VehicleLib[vehicle_id] = vehicleclass

                VehicleLib[vehicle_id].update()
        print(junction_counts)
    def Vehicle_Control(self):
        for laneID in self.lane_ids:
            for (idx,vehicle_id) in enumerate(self.vehicle_ids[laneID]):
                if vehicle_id[0:3] == 'CAV':
                    #print(simulate_info.now_time)
                    VehicleLib[vehicle_id].acceleration_control(simulate_info.now_time,dt,self.id,laneID)

    def Conv_from_dict(self, dict,Simpart = False):
        self.vehicle_num = dict['vehicle_num']
        self.lanes_length = dict['lanes_length']
        self.vehicle_ids_old = self.vehicle_ids.copy()
        self.vehicle_ids = dict['vehicle_ids']
        if Simpart == True:
            vehicle_id_list_old = []
            vehicle_id_list_new = []
            for lane_id in self.lane_ids:
                if lane_id[-1:] == '0':
                    continue
                vehicle_id_list_old += self.vehicle_ids_old[lane_id]
                vehicle_id_list_new += self.vehicle_ids[lane_id]
            junction_counts[self.id] += len(set(vehicle_id_list_old) - set(vehicle_id_list_new))

 
    def Conv_to_dict(self):
        return {
            'vehicle_num': self.vehicle_num,
            'lanes_length': self.lanes_length,
            'vehicle_ids': self.vehicle_ids
        }


LightLib = {}
class Light():
    def __init__(self,id):
        self.id = id #
        self.lane_ids = traffic_light_to_lanes[self.id]
        self.phase = {k:'' for k in self.lane_ids} #'r' 'g' 'y'
        self.remaining_time = {k:0 for k in self.lane_ids}
        self.nextphase = {k:'' for k in self.lane_ids}

        # 通过订阅的方式，避免频繁traci
        traci.trafficlight.subscribe(self.id, [traci.constants.TL_NEXT_SWITCH, traci.constants.TL_RED_YELLOW_GREEN_STATE])
        self.completeRedYellowGreenDefinition = traci.trafficlight.getCompleteRedYellowGreenDefinition(self.id)
        self.controlled_lanes = traci.trafficlight.getControlledLanes(self.id)
        self.programID = traci.trafficlight.getProgram(self.id)
        self.phaselist = {self.completeRedYellowGreenDefinition[0].phases[index].state:index for index in range(len(self.completeRedYellowGreenDefinition[0].phases))}
        self.remaining_time_common = 0
        self.update()

    def update(self):
        tl_subscription = traci.trafficlight.getSubscriptionResults(self.id)
        next_switch_time = tl_subscription[traci.constants.TL_NEXT_SWITCH]
        current_phase_state = tl_subscription[traci.constants.TL_RED_YELLOW_GREEN_STATE]
        remaining_time = next_switch_time - simulate_info.now_time

        for idx, lane_id in enumerate(self.controlled_lanes):
            phase = current_phase_state[idx].lower()
            self.phase[lane_id] = phase
            self.remaining_time[lane_id] = max(remaining_time, 0)  # 统一剩余时间
            if self.phase[lane_id] == 'r':
                self.nextphase[lane_id] = 'g'
            elif self.phase[lane_id] == 'g':
                self.nextphase[lane_id] = 'y'
            else:
                self.nextphase[lane_id] = 'r'
        self.remaining_time_common = remaining_time
        self.nowphase_index = self.phaselist[current_phase_state]
        self.current_phase_state = current_phase_state
        self.next_phase_state = self.completeRedYellowGreenDefinition[0].phases[(self.nowphase_index + 1) % 8].state
        self.next_3_phase_state = self.completeRedYellowGreenDefinition[0].phases[(self.nowphase_index + 3) % 8].state

    def Conv_from_dict(self, dict):
        self.phase = dict['phase']
        self.remaining_time = dict['remaining_time']
        self.nextphase = dict['nextphase']
        self.remaining_time_common = dict['remaining_time_common']
    def Conv_to_dict(self):
        return {
            'phase': self.phase,
            'remaining_time': self.remaining_time,
            'nextphase': self.nextphase,
            'remaining_time_common': self.remaining_time_common
        }