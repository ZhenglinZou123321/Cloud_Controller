import traci
# Traci远程连接的设置
traci.init(port = 14491,host="192.168.100.103")
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
r.flushdb()
print('已清空redis')

# 获取当前脚本所在的目录
script_dir = os.path.dirname(os.path.abspath(__file__))


# 创建交通数据记录表
Record_Gap = 60/Global_Vars.dt
with open('traffic_data_gaussian.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['time', 'road_id', 'vehicle_count', 'average_speed'])

with open('Travel_time_datas.csv',mode='w',newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['vehicle_id','Travel_time'])

with open('Waiting_time_datas.csv',mode='w',newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['vehicle_id','Waiting_time'])

lane_keys = Global_Vars.lane_index_dict.keys()

lane_keys = [i for i in lane_keys if not i.startswith(":")]

lane_statistic = {}
lane_count = {}
lane_average_speed = {}
for lane_key in lane_keys:
    lane_statistic[lane_key]={}
    lane_count[lane_key] = 0
    lane_average_speed[lane_key] = 0

# per time
# lane_statistic[lane_key][time]={'vehicle_count':0,'average_speed':0}

Light_threads = []

if __name__ == '__main__':

    for lane , length in Global_Vars.Lanes_length.items():
        r.set(lane,length)


    r.set('N',pickle.dumps(Global_Vars.N))
    r.set('dt',pickle.dumps(Global_Vars.dt))
    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    for junc in Global_Vars.Intelligent_Sigal_List:
        controller = TrafficLightController(junc,Global_Vars.traffic_light_to_lanes,Global_Vars.lane_index_dict,Global_Vars.lane_adj_matrix,Global_Vars.N,Global_Vars.dt,Global_Vars.L_safe,device=device)
        lightclass = Global_Vars.Light(junc)
        juncclass = Global_Vars.Junc(junc)
        Global_Vars.LightLib[junc] = lightclass
        Global_Vars.JuncLib[junc] = juncclass
        controller.start()
        Light_threads.append(controller)

    '''lights_packed = msgpack.packb(Global_Vars.LightLib, use_bin_type=True)
    r.set("LightLib",lights_packed)'''
    
    '''juncs_packed = msgpack.packb(Global_Vars.JuncLib, use_bin_type=True)
    r.set("JuncLib",juncs_packed)'''

    print(Global_Vars.LightLib)
    LightLib_dict = {}
    VehicleLib_dict = {}
    Travel_Time = {}
    Waiting_Time = {}

    while traci.simulation.getMinExpectedNumber() > 0:
        start_time = time.time()
        print(Global_Vars.junction_counts)
        Global_Vars.simulate_info.update()
        simulate_info_packed = msgpack.packb(Global_Vars.simulate_info.__dict__, use_bin_type=True)
        r.set("simulate_info",simulate_info_packed)

        print(f'仿真信息更新 time {time.time()-start_time}')
        start_time = time.time()
        try:# 在junction_terminal中，用了hash的形式存储junc，因此JuncLib是hash类型的。
            # hash类型就允许我们在数据库里单独取这个字典中的某一项进行读写
            JuncLib_dict = r.hgetall("JuncLib")
            for key in JuncLib_dict.keys():
                key_str = key.decode()
                Global_Vars.JuncLib[key_str].Conv_from_dict(msgpack.unpackb(JuncLib_dict[key], raw=False),Simpart = True)
        except:
            print("未能从redis读取JuncLib")
        print(f'JuncLib 更新 time {time.time()-start_time}')
        # 对于LightLib，是将整个字典变成字符串形式存储的，所以是string形式
        # 不能够单独对其中某一项进行读写，只能将其全部下载下来，再全部上传来更新
        start_time = time.time()
        for junc in Global_Vars.Intelligent_Sigal_List:
            Global_Vars.LightLib[junc].update()
            LightLib_dict[junc] = Global_Vars.LightLib[junc].Conv_to_dict()  
        lights_packed = msgpack.packb(LightLib_dict, use_bin_type=True)
        r.set("LightLib",lights_packed)
        print(f'LightLib更新 time {time.time()-start_time}')

        try:
            Global_Vars.Vehicle_IDs = set(msgpack.unpackb(r.get("Vehicle_IDs"), raw=False))
        except:
            print("未能从redis读取Vehicle_IDs")

        start_time = time.time()
        to_remove = set()
        for vehicle_id in Global_Vars.Vehicle_IDs:
            if vehicle_id not in Global_Vars.VehicleLib:
                vehicleclass = Global_Vars.Vehicle(vehicle_id,vehicle_id[0:3])
                Global_Vars.VehicleLib[vehicle_id] = vehicleclass
            Global_Vars.VehicleLib[vehicle_id].update()

            if Global_Vars.VehicleLib[vehicle_id].speed <=0:
                if vehicle_id not in Waiting_Time.keys():
                    Waiting_Time[vehicle_id] = 1
                else:
                    Waiting_Time[vehicle_id] += 1

            if Global_Vars.VehicleLib[vehicle_id].running == False:
                Global_Vars.VehicleLib[vehicle_id].time_exit = Global_Vars.simulate_info.now_time
                Travel_Time[vehicle_id] = Global_Vars.VehicleLib[vehicle_id].time_exit - Global_Vars.VehicleLib[vehicle_id].time_entry
                to_remove.add(vehicle_id)
                del Global_Vars.VehicleLib[vehicle_id]
                continue
            
            VehicleLib_dict[vehicle_id] = Global_Vars.VehicleLib[vehicle_id].Conv_to_dict() 
            if Global_Vars.step % Record_Gap == 0:
                if Global_Vars.VehicleLib[vehicle_id].lane[0:1] == ':':
                    continue
                lane_average_speed[Global_Vars.VehicleLib[vehicle_id].lane] = (lane_average_speed[Global_Vars.VehicleLib[vehicle_id].lane]*lane_count[Global_Vars.VehicleLib[vehicle_id].lane] + Global_Vars.VehicleLib[vehicle_id].speed)/(lane_count[Global_Vars.VehicleLib[vehicle_id].lane]+1)
                lane_count[Global_Vars.VehicleLib[vehicle_id].lane] += 1
        Global_Vars.Vehicle_IDs -= to_remove  
        print(f'Vehicle信息更新 time {time.time()-start_time}') 
         
        r.set("Vehicle_IDs",msgpack.packb(list(Global_Vars.Vehicle_IDs), use_bin_type=True))

        VehicleLib_packed = msgpack.packb(VehicleLib_dict, use_bin_type=True)
        r.set("VehicleLib",VehicleLib_packed)

        if Global_Vars.step % Record_Gap == 0:
            # 写入CSV文件
            with open('traffic_data_gaussian.csv', mode='a', newline='') as file:
                writer = csv.writer(file)
                for lane_key in lane_keys:
                    lane_statistic[lane_key][Global_Vars.step]={'vehicle_count':lane_count[lane_key],'average_speed':lane_average_speed[lane_key]}
                    lane_count[lane_key] = 0
                    lane_average_speed[lane_key] = 0
                    for sstep in lane_statistic[lane_key].keys():
                        writer.writerow([sstep, lane_key, lane_statistic[lane_key][sstep]['vehicle_count'], lane_statistic[lane_key][sstep]['average_speed']])
                    lane_statistic[lane_key]={}
            with open('Travel_time_datas.csv',mode='w',newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['vehicle_id','Travel_time'])
                for vehicle_id in Travel_Time.keys():
                    writer.writerow([vehicle_id,Travel_Time[vehicle_id]])
                    
            with open('Waiting_time_datas.csv',mode='w',newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['vehicle_id','Waiting_time'])
                for vehicle_id in Waiting_Time.keys():
                    writer.writerow([vehicle_id,Waiting_Time[vehicle_id]])
                
            # per time
            # lane_statistic[lane_key][time]={'vehicle_count':0,'average_speed':0}
            merge_and_save_memory([Light_threads[i].agent for i in range(len(Global_Vars.Intelligent_Sigal_List))],'datas.json')

        Global_Vars.step += 1
        traci.simulationStep()


        



