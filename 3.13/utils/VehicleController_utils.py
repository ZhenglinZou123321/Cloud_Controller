import threading
import traci
import copy
from Solver_utils import *
# 车辆参数
'''MAX_SPEED = 11  # 最大速度 (m/s)
MIN_SPEED = 0
MAX_ACCEL = 3  # 最大加速度 (m/s^2)
MIN_ACCEL = -20

# IDM模型参数
V_0 = MAX_SPEED  # 期望速度 (m/s)，可根据实际情况调整
T = 1.5  # 期望车头时距 (s)，可根据实际情况调整
a_max = MAX_ACCEL  # 最大加速度 (m/s²)，与前面已定义的保持一致或按需调整
b = -1*MIN_ACCEL  # 舒适制动减速度 (m/s²)，可根据实际情况调整
s_0 = 2  # 最小间距 (m)，可根据实际情况调整
delta = 4  # 速度影响指数参数，可根据实际情况调整'''



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
    s_star = s_0 + current_speed * T + (current_speed * relative_speed) / (2 * np.sqrt(a_max * b))
    acceleration = a_max * (1 - (current_speed / V_0) ** delta - (s_star / gap) ** 2)
    return min(max(acceleration,MIN_ACCEL),MAX_ACCEL)



def get_remaining_phase_and_time(lane_id): #获取信号灯当前相位和剩余时间
    # 按照固定字符进行分割
    x, rest = lane_id.split("t", 1)  # 分割出 X 和剩余部分
    intersection_id, z = rest.split("_", 1)  # 分割出 Y 和 Z
    # 获取当前仿真时间
    current_time = traci.simulation.getTime()
    # 获取下一个信号切换的时间
    next_switch_time = traci.trafficlight.getNextSwitch(intersection_id)
    # 计算剩余时间 秒
    remaining_time = next_switch_time - current_time
    current_phase = traci.trafficlight.getRedYellowGreenState(intersection_id)[traci.trafficlight.getControlledLanes(intersection_id).index(lane_id)]
    return current_phase.lower(),max(remaining_time, 0)  # 防止负值




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
        global step
        while self.running and traci.simulation.getMinExpectedNumber() > 0 and self.vehicle_id[0:3] == "CAV":
            print(f"{self.vehicle_id[0:3]} thread running")
            try:
                # 检查车辆是否仍然在仿真中
                try:
                    # 尝试获取车辆位置
                    position = traci.vehicle.getPosition(self.vehicle_id)
                except traci.exceptions.TraCIException:
                    print(f"在第 {step} 步，车辆 {self.vehicle_id} 不在场景中。")
                    self.running = False
                    continue

                # 获取本车速度
                self.speed = traci.vehicle.getSpeed(self.vehicle_id)
                # 获取前车信息
                self.front_info = traci.vehicle.getLeader(self.vehicle_id)        
                self.idm_acc = None
                if self.front_info != None:
                    self.front_id,self.gap = self.front_info
                    print(f"{self.vehicle_id}的前车是{self.front_id}")

                # 有前车的情况下，计算idm加速度
                if self.front_info != None and traci.vehicle.getLaneID(self.vehicle_id) == traci.vehicle.getLaneID(self.front_id):
                    self.idm_acc = idm_acceleration(self.speed, traci.vehicle.getSpeed(self.front_id), self.gap, front_vehicle_id=None)
                else:
                    # 无前车的情况下，或者前车与本车不在一个车道时
                    self.lane_now = traci.vehicle.getLaneID(self.vehicle_id)
                    self.phase,self.remaining_time = get_remaining_phase_and_time(self.lane_now)
                    if self.phase == 'r' or self.phase == 'y':
                        self.lane_length = traci.lane.getLength(self.lane_now)
                        self.idm_acc = idm_acceleration(self.speed, 0.0,
                                                    self.lane_length-traci.vehicle.getLanePosition(self.vehicle_id),
                                                    front_vehicle_id=None)
                        
                # 速度施加环节
                if self.vehicle_id[0:3] == "CAV":
                    self.control_signal = list(self.control_signal_new)
                    if len(self.control_signal) != 0:
                        traci.vehicle.setSpeedMode(self.vehicle_id, 00000)  # 关闭跟驰模型
                        self.mpc_acc = self.get_control_signal(step, self.dt)

                        # 如果有idm控制量
                        if self.idm_acc != None:
                            print(f"{self.vehicle_id} MPC控制量为：{self.mpc_acc}, IDM控制量为：{self.idm_acc}")
                            if self.mpc_acc != None:
                                if self.mpc_acc < self.idm_acc:
                                    self.acc_control = self.mpc_acc
                                    print(f"{self.vehicle_id} 施加MPC控制量为：{self.acc_control}")
                                    traci.vehicle.setColor(self.vehicle_id, (0, 0, 255)) # MPC = blue
                                else:
                                    self.acc_control = self.idm_acc
                                    print(f"{self.vehicle_id} 施加IDM控制量为：{self.acc_control}")
                                    traci.vehicle.setColor(self.vehicle_id, (255, 0, 0))  # IDM = red
                            else:
                                self.acc_control = self.idm_acc
                                print(f"{self.vehicle_id} 施加IDM控制量为：{self.acc_control}")
                                traci.vehicle.setColor(self.vehicle_id, (255, 0, 0))  # IDM = red
                        else:
                            # 如果没有idm控制量，则施加mpc控制量
                            self.acc_control = self.mpc_acc
                            print(f"{self.vehicle_id} 施加MPC控制量为：{self.acc_control}")
                            traci.vehicle.setColor(self.vehicle_id, (0, 0, 255))  # MPC =  blue

                        # 如果经过上述操作后，得到了一个加速度控制量
                        if self.acc_control != None:
                            traci.vehicle.setAcceleration(self.vehicle_id, self.acc_control, 1)
                        else:
                            traci.vehicle.setAcceleration(self.vehicle_id, 0.0, 1)
                        print(f"{self.vehicle_id}已施加加速度控制量：{self.acc_control}")
            except:
                print(f"{self.vehicle_id} 施加加速度控制量失败")         


    def stop(self):
        self.running = False