import threading
import traci
import copy
from utils.Solver_utils import *
import Global_Vars
import traceback
import re



def idm_acceleration(current_speed, front_vehicle_speed, gap,  front_vehicle_id=None):
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
    s_star = Global_Vars.s_0 + current_speed * Global_Vars.T + (current_speed * relative_speed) / (2 * np.sqrt(Global_Vars.a_max * Global_Vars.b))
    acceleration = Global_Vars.a_max * (1 - (current_speed / Global_Vars.V_0) ** Global_Vars.delta - (s_star / gap) ** 2)
    return min(max(acceleration,Global_Vars.MIN_ACCEL),Global_Vars.MAX_ACCEL)



def lane_is_out_junction(s):
    pattern = r'^j.*tj.*$'
    if re.match(pattern, s):
        return True
    return False



class VehicleController(threading.Thread):
    def __init__(self, vehicle_id_,dt):
        threading.Thread.__init__(self)
        self.vehicle_id = vehicle_id_
        self.running = True
        self.acc_control = 0
        self.idm_acc = 0
        self.control_signal = []
        self.control_signal_new = []
        self.time_when_cal = 0
        self.dt = dt
    def get_control_signal(self,time_now, dt):
        if int((time_now-self.time_when_cal)/dt) <= len(self.control_signal)-1:
            return self.control_signal[int((time_now-self.time_when_cal)/dt)]
        else:
            return None

    def run(self):

        while self.running:
            ##print(f"{self.vehicle_id[0:3]} thread running")

            # 车流量计数
            # 获取车辆当前所在的 edge
            current_edge = Global_Vars.VehicleLib[self.vehicle_id].RoadID
            # 如果车辆在网络的有效 edge 上
            if current_edge and current_edge[0] != ":":
                # 获取车辆的上一个 edge
                previous_edge = Global_Vars.previous_vehicle_edges.get(self.vehicle_id)
                # 检查车辆是否从一个 edge 转移到另一个 edge
                if previous_edge and previous_edge != current_edge:
                    # 获取 previous_edge 的终点 junction
                    to_junction = Global_Vars.net.getEdge(previous_edge).getToNode().getID()
                    # 在 junction_counts 中递增计数
                    if to_junction in Global_Vars.junction_counts:
                        Global_Vars.junction_counts[to_junction] += 1
                    else:
                        Global_Vars.junction_counts[to_junction] = 1
                # 更新车辆的上一个 edge 为当前 edge
                Global_Vars.previous_vehicle_edges[self.vehicle_id] = current_edge

            try:
                # 检查车辆是否仍然在仿真中
                if Global_Vars.VehicleLib[self.vehicle_id].running == False:
                    traceback.print_exc()
                    #print(f"在第 {Global_Vars.step} 步，车辆 {self.vehicle_id} 不在场景中。")
                    self.running = False
                    continue

                # 获取本车速度
                self.speed = Global_Vars.VehicleLib[self.vehicle_id].speed
                # 获取前车信息
                self.front_info = Global_Vars.VehicleLib[self.vehicle_id].leader       
                self.idm_acc = None
                if self.front_info != None:
                    self.front_id,self.gap = self.front_info
                    #print(f"{self.vehicle_id}的前车是{self.front_id}")

                # 有前车的情况下，计算idm加速度
                if self.front_info != None and Global_Vars.VehicleLib[self.vehicle_id].lane == Global_Vars.VehicleLib[self.front_id].lane:
                    self.idm_acc = idm_acceleration(self.speed, Global_Vars.VehicleLib[self.front_id].speed, self.gap, front_vehicle_id=None)
                else:
                    # 无前车的情况下，或者前车与本车不在一个车道时
                    self.lane_now = Global_Vars.VehicleLib[self.vehicle_id].lane
                    # 如果车辆目前在路口外
                    if lane_is_out_junction(self.lane_now):
                        # 如果该路口是智能信号灯路口
                        if Global_Vars.lane_to_traffic_light[self.lane_now] in Global_Vars.Intelligent_Sigal_List:
                            self.phase = Global_Vars.LightLib[Global_Vars.lane_to_traffic_light[self.lane_now]].phase[self.lane_now]
                            self.remaining_time =  Global_Vars.LightLib[Global_Vars.lane_to_traffic_light[self.lane_now]].remaining_time[self.lane_now]
                            #get_remaining_phase_and_time(self.lane_now)
                            if self.phase == 'r' or self.phase == 'y':
                                self.lane_length = Global_Vars.Lanes_length[self.lane_now]
                                self.idm_acc = idm_acceleration(self.speed, 0.0,
                                                            self.lane_length-Global_Vars.VehicleLib[self.vehicle_id].laneposition,
                                                            front_vehicle_id=None)
                        # 如果该路口不是智能信号灯路口
                        #*********************************************
                    # 如果车辆目前在路口内
                    #********************************************

                        
                # 速度施加环节
                if self.vehicle_id[0:3] == "CAV":
                    self.control_signal = list(self.control_signal_new)
                    if len(self.control_signal) != 0:
                        traci.vehicle.setSpeedMode(self.vehicle_id, 00000)  # 关闭跟驰模型
                        self.mpc_acc = self.get_control_signal(Global_Vars.step, self.dt)

                        # 如果有idm控制量
                        if self.idm_acc != None:
                            #print(f"{self.vehicle_id} MPC控制量为：{self.mpc_acc}, IDM控制量为：{self.idm_acc}")
                            if self.mpc_acc != None:
                                if self.mpc_acc < self.idm_acc:
                                    self.acc_control = self.mpc_acc
                                    #print(f"{self.vehicle_id} 施加MPC控制量为：{self.acc_control}")
                                    traci.vehicle.setColor(self.vehicle_id, (0, 0, 255)) # MPC = blue
                                else:
                                    self.acc_control = self.idm_acc
                                    #print(f"{self.vehicle_id} 施加IDM控制量为：{self.acc_control}")
                                    traci.vehicle.setColor(self.vehicle_id, (255, 0, 0))  # IDM = red
                            else:
                                self.acc_control = self.idm_acc
                                #print(f"{self.vehicle_id} 施加IDM控制量为：{self.acc_control}")
                                traci.vehicle.setColor(self.vehicle_id, (255, 0, 0))  # IDM = red
                        else:
                            # 如果没有idm控制量，则施加mpc控制量
                            self.acc_control = self.mpc_acc
                            #print(f"{self.vehicle_id} 施加MPC控制量为：{self.acc_control}")
                            traci.vehicle.setColor(self.vehicle_id, (0, 0, 255))  # MPC =  blue

                        # 如果经过上述操作后，得到了一个加速度控制量
                        if self.acc_control != None:
                            traci.vehicle.setAcceleration(self.vehicle_id, self.acc_control, 1)
                        else:
                            traci.vehicle.setAcceleration(self.vehicle_id, 0.0, 1)
                        #print(f"{self.vehicle_id}已施加加速度控制量：{self.acc_control}")
            except:
                traceback.print_exc()
                #print(f"{self.vehicle_id} 施加加速度控制量失败")         


    def stop(self):
        self.running = False