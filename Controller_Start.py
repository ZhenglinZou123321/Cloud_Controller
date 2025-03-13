import traci
import sumolib
import threading
import numpy as np
import os
import json
import pandas as pd
import csv



# 获取当前脚本所在的目录
script_dir = os.path.dirname(os.path.abspath(__file__))

# Traci远程连接的设置
traci.init(port = 14491,host="192.168.100.104")
print('111')
traci.setOrder(1) #设置优先级，数字越大，越高

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

# 创建交通数据记录表
with open('traffic_data_gaussian.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['time', 'road_id', 'vehicle_count', 'average_speed'])

# 所要管控的路口id
Intelligent_Sigal_List = ['j5', 'j6', 'j7', 'j10','j11','j12']

# 车辆线程字典
vehicle_threads = {}

# 仿真参数
step = 0 # 仿真步

if __name__ == '__main__':

    # 路口线程创建
    #******************************


    # 信号灯线程创建
    #******************************


    # 开始仿真
    print("ready to start")

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        # 车辆线程创建与关闭
        vehicle_ids = traci.vehicle.getIDList()
        for vehicle_id in vehicle_ids:
            if vehicle_id not in vehicle_threads:
                # 如果发现新CAV，启动一个控制线程
                # 线程的关闭和清除，放在线程类里
                if vehicle_id[0:3] == "CAV":
                    traci.vehicle.setColor(vehicle_id, (255, 0, 0))
                    controller = VehicleController(vehicle_id)
                    controller.start()
                    vehicle_threads[vehicle_id] = controller
                    print(f"Started thread for vehicle {vehicle_id}")
                    continue

        # Step 仿真步的记录
        step += 1



