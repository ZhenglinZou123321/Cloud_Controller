import traci
import sumolib
import threading
import numpy as np
import os
import json
import pandas as pd
import csv
from utils.VehicleController_utils import *
from utils.Solver_utils import *
from utils.junction_utils import *
from utils.traffic_light import TrafficLightController
import Global_Vars

'''
dt = 0.2
N=40
L_safe = 4 + 3 #4米车长，3米间距'''

#详细参数在Solver_utils中统一设置

'''
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
vehicles = []'''



# 获取当前脚本所在的目录
script_dir = os.path.dirname(os.path.abspath(__file__))

# Traci远程连接的设置
traci.init(port = 14491,host="192.168.100.104")
print('111')
traci.setOrder(1) #设置优先级，数字越大，越高




# 创建交通数据记录表
with open('traffic_data_gaussian.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['time', 'road_id', 'vehicle_count', 'average_speed'])

# 所要管控的路口id
Intelligent_Sigal_List = ['j5', 'j6', 'j7', 'j10','j11','j12']





if __name__ == '__main__':

    # 路口线程创建
    #******************************
    for junc in Intelligent_Sigal_List:
        controller = JunctionController(junc,Global_Vars.traffic_light_to_lanes,N,dt,L_safe)
        controller.start()

    # 信号灯线程创建
    #******************************
    for junc in Intelligent_Sigal_List:
        controller = TrafficLightController(junc,Global_Vars.traffic_light_to_lanes,Global_Vars.lane_index_dict,Global_Vars.lane_adj_matrix,N,dt,L_safe)
        controller.start()
        
    # 开始仿真
    print("ready to start")

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        # 车辆线程创建与关闭
        vehicle_ids = traci.vehicle.getIDList()
        for vehicle_id in vehicle_ids:
            if vehicle_id not in Global_Vars.vehicle_threads:
                # 如果发现新CAV，启动一个控制线程
                # 线程的关闭和清除，放在线程类里
                if vehicle_id[0:3] == "CAV":
                    traci.vehicle.setColor(vehicle_id, (255, 0, 0))
                    controller = VehicleController(vehicle_id,dt)
                    controller.start()
                    Global_Vars.vehicle_threads[vehicle_id] = controller
                    print(f"Started thread for vehicle {vehicle_id}")
                    continue

        # Step 仿真步的记录
        Global_Vars.step += 1



