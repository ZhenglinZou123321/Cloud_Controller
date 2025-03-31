import traci
import threading
import time
import copy
import numpy as np
from utils.Solver_utils import *
import Global_Vars


class JunctionController():
    def __init__(self, junction_id_,traffic_light_to_lanes_,N,dt,L_safe):
        self.junction_id = junction_id_
        self.running = True
        self.acc_control = 0
        self.idm_acc = 0
        self.control_signal = []
        self.time_when_cal = 0
        self.traffic_light_to_lanes = traffic_light_to_lanes_
        self.last_quarter_vehicles = {}
        self.N = N
        self.dt = dt
        self.L_safe = L_safe
        self.log_file = f"logs/junction_{self.junction_id}.log"

    def get_last_quarter_every_lane(self):
        lane_ids = self.traffic_light_to_lanes[self.junction_id]
        lane_vehicles = {}
        for laneID in lane_ids:
            # 获取当前车道上的车辆
            lane_vehicles[laneID] = list(Global_Vars.JuncLib[self.junction_id].vehicle_ids[laneID])
            # 获取车道长度
            lane_length = Global_Vars.Lanes_length[laneID]
            # 筛选后四分之一段的车辆
            vehicles_in_last_quarter = [
                vehID for vehID in lane_vehicles[laneID]
                if Global_Vars.VehicleLib[vehID].laneposition > 0.5 * lane_length
            ]
            self.last_quarter_vehicles[laneID] = vehicles_in_last_quarter #idx=0的车是最靠近lane终点的，以此类推

    def update_cav_speeds(self):
        start_time = time.time()
        now_time = Global_Vars.simulate_info.now_time
        for lane_id in self.traffic_light_to_lanes[self.junction_id]:
            num_CAV = 0
            num_HDV = 0
            initial_state_CAV = []  # 最后要转换为np.array() [位置,速度]
            initial_state_HDV = []  # 最后要转换为np.array() [位置,速度]
            CAV_id_list = []
            HDV_id_list = []
            type_list = []

            vehicles_list_this_lane = copy.deepcopy(self.last_quarter_vehicles[lane_id])
            for vehicle_id in vehicles_list_this_lane:

                if vehicle_id[0:3] == "CAV":
                    Global_Vars.VehicleLib[vehicle_id].time_when_cal = now_time
                    Global_Vars.VehicleLib[vehicle_id].control_signal_new = []
                    num_CAV +=1
                    type_list.append('CAV')
                    state = [Global_Vars.VehicleLib[vehicle_id].laneposition, Global_Vars.VehicleLib[vehicle_id].speed]
                    initial_state_CAV.append(state)
                    CAV_id_list.append(vehicle_id)
                elif vehicle_id[0:3] == "HDV":
                    num_HDV +=1
                    type_list.append('HDV')
                    state = [Global_Vars.VehicleLib[vehicle_id].laneposition, Global_Vars.VehicleLib[vehicle_id].speed]
                    initial_state_HDV.append(state)
                    HDV_id_list.append(vehicle_id)
                #CAV HDV的状态都要收集 但是只控制CAV
                    #target_speed = MAX_SPEED if is_green_light("j3") else 0
                    #optimized_speed, optimized_accel = optimize_speed(vehicle_id, target_speed, MAX_SPEED, MAX_ACCEL, REACTION_TIME)
            if len(initial_state_CAV) != 0:
                #print('2')
                type_info = (type_list,num_CAV,num_HDV)
                initial_state_CAV = np.array(initial_state_CAV)
                initial_state_HDV = np.array(initial_state_HDV)

                #mpc_control(initial_state, 0, weights=[1.0,0.5], N=20, dt=dt, bounds=(MIN_ACCEL,MAX_ACCEL,0,MAX_SPEED),type_info=type_info,now_lane = lane_id,lane_towards = lane_towards,last_quarter_vehicles=last_quarter_vehicles)
                solve_status,u = QP_solver(initial_state_CAV, initial_state_HDV, vehicles_list_this_lane, Global_Vars.N, Global_Vars.dt, v_max=Global_Vars.MAX_SPEED, v_min=Global_Vars.MIN_SPEED, a_max=Global_Vars.MAX_ACCEL,a_min=Global_Vars.MIN_ACCEL, L_safe=Global_Vars.L_safe, lane_now=lane_id, CAV_id_list=CAV_id_list, HDV_id_list=HDV_id_list,intersection_id=self.junction_id)

                if solve_status:
                    i = 0 
                    for vehicle in CAV_id_list:
                        Global_Vars.VehicleLib[vehicle].control_signal_new = u[i::num_CAV]
                        i += 1
                #print('3')
        end_time = time.time()

    def run(self):
        self.get_last_quarter_every_lane()
        self.update_cav_speeds()

