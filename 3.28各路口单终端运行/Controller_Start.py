import traci
# Traci远程连接的设置
traci.init(port = 14491,host="192.168.100.123")
traci.setOrder(0) #设置优先级，数字越小，越高
#print('111')
import time
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
from utils.traffic_light import *
import Global_Vars
import msgpack
import redis
r = redis.Redis(host='192.168.100.21', port=6379, db=0)

# 获取当前脚本所在的目录
script_dir = os.path.dirname(os.path.abspath(__file__))


# 创建交通数据记录表
with open('traffic_data_gaussian.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['time', 'road_id', 'vehicle_count', 'average_speed'])


if __name__ == '__main__':

    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    for junc in Global_Vars.Intelligent_Sigal_List:
        controller = TrafficLightController(junc,Global_Vars.traffic_light_to_lanes,Global_Vars.lane_index_dict,Global_Vars.lane_adj_matrix,Global_Vars.N,Global_Vars.dt,Global_Vars.L_safe,device=device)
        lightclass = Global_Vars.Light(junc)
        juncclass = Global_Vars.Junc(junc)
        Global_Vars.LightLib[junc] = lightclass
        Global_Vars.JuncLib[junc] = juncclass
        controller.start()

    lights_packed = msgpack.packb(Global_Vars.LightLib, use_bin_type=True)
    r.set("LightLib",lights_packed)
    
    juncs_packed = msgpack.packb(Global_Vars.JuncLib, use_bin_type=True)
    r.set("JuncLib",juncs_packed)

    print(Global_Vars.LightLib)

    while traci.simulation.getMinExpectedNumber() > 0:

        Global_Vars.simulate_info.update()
        simulate_info_packed = msgpack.packb(Global_Vars.simulate_info, use_bin_type=True)
        r.set("simulate_info",simulate_info_packed)

        for junc in Global_Vars.Intelligent_Sigal_List:
            Global_Vars.LightLib[junc].update()   
        lights_packed = msgpack.packb(Global_Vars.LightLib.__dict__, use_bin_type=True)
        r.set("LightLib",lights_packed)

        try:
            Global_Vars.Vehicle_IDs = msgpack.unpackb(r.get("Vehicle_IDs"), raw=False)
        except:
            print("未能从redis读取Vehicle_IDs")
        for vehicle_id in Global_Vars.Vehicle_IDs:
            if vehicle_id not in Global_Vars.VehicleLib:
                vehicleclass = Global_Vars.Vehicle(vehicle_id,vehicle_id[0:3])
                Global_Vars.VehicleLib[vehicle_id] = vehicleclass
            Global_Vars.VehicleLib[vehicle_id].update()

        VehicleLib_packed = msgpack.packb(Global_Vars.VehicleLib.__dict__, use_bin_type=True)
        r.set("VehicleLib",VehicleLib_packed)

        Global_Vars.step += 1
        traci.simulationStep()


        



