import traci
import threading
import time
import copy
import numpy as np
from utils.Solver_utils import *
import torch
import torch.nn as nn
import random
from collections import deque
import torch.optim as optim
import Global_Vars





state_size = 1+3*3+3*3+1+3*3+3*3+2
Action_list = [10,15,20,25,30,35,40,45,50,55,60,65,70,75,80]

shared_model = True
shared_model_location = 'models/agent_model.pth'
Train_Or_Not = False

Adaptive_Or_Not = False # 是否自适应调节信号相位，False就是固定相位，但也会记录experience

Least_Check_Time = 3

train_gap = 20
train_batchsize = 32


# 合并所有智能体的 memory
def merge_and_save_memory(agent_list, file_path):
    merged_memory = []

    for agent in agent_list:  # 遍历每个智能体
        for experience in agent.memory:
            state, action, reward, next_state = experience
            merged_memory.append({
                'state': state.tolist(),  # 将 numpy 数组转换为列表
                'action': action,
                'reward': reward,
                'next_state': next_state.tolist(),  # 将 numpy 数组转换为列表
            })

    # 保存合并后的 memory 到 JSON 文件
    with open(file_path, 'a') as file:
        json.dump(merged_memory, file)

def match_strings(str1, str2):
    # 使用正则表达式将字符串按 "tj" 分成三部分：j开头部分、中间部分、t结尾部分
    pattern = r'j(\d+)tj(\d+)'
    match1 = re.match(pattern, str1)
    match2 = re.match(pattern, str2)

    # 检查是否匹配成功，并且两者的分隔结果相同（忽略顺序）
    if match1 and match2:
        part1_1, part2_1 = match1.groups()
        part1_2, part2_2 = match2.groups()
        # 比较是否相同
        return (part1_1 == part2_2 and part2_1 == part1_2) or (part1_1 == part1_2 and part2_1 == part2_2)
    return False



def get_remaining_phase_time(traffic_light_id): #获取信号灯剩余时间
    # 获取当前仿真时间
    current_time = traci.simulation.getTime()
    # 获取下一个信号切换的时间
    next_switch_time = traci.trafficlight.getNextSwitch(traffic_light_id)
    # 计算剩余时间
    remaining_time = next_switch_time - current_time
    return max(remaining_time, 0)  # 防止负值


traffic_signal_dict = {'r':0,'g':1,'y':2}
'''
def get_lane_state(lane_id,lane_dict,lane_m):

    traffic_signal_dict = {'r':0,'g':1,'y':2}
    edge_id = traci.lane.getEdgeID(lane_id)
    to_junction = traci.edge.getToJunction(edge_id)
    if to_junction in Global_Vars.traffic_light_to_lanes.keys():
        controlled_lanes = traci.trafficlight.getControlledLanes(to_junction)
        current_phase_state = traci.trafficlight.getRedYellowGreenState(to_junction)
        lane_index = controlled_lanes.index(lane_id)
        lane_phase = current_phase_state[lane_index]
        remain_time = get_remaining_phase_time(to_junction)
        return traffic_signal_dict[lane_phase.lower()], remain_time
    else:
        lane_phase = 'g'
        remain_time = 99

    return traffic_signal_dict[lane_phase],remain_time'''


def get_state(intersection_id,lane_index_dict,lane_adj):
    next_state_of_last = []
    new_state = []
    traffic_signal_dict = {'r':0,'g':1,'y':2}
    checked_lane = []
    dentisy_self = 0#不处理右转车道
    dentisy_from = 0
    dentisy_to = 0#目标车道的车辆密度
    next_green_density_last = []
    next_green_density_new = []
    current_phase_state = Global_Vars.LightLib[intersection_id].current_phase_state
    #current_phase_state = traci.trafficlight.getRedYellowGreenState(intersection_id)
    next_phase_state = Global_Vars.LightLib[intersection_id].next_phase_state
    next_3_phase_state = Global_Vars.LightLib[intersection_id].next_3_phase_state
    #next_phase_state = traci.trafficlight.getCompleteRedYellowGreenDefinition(intersection_id)[0].phases[(nowphase_index + 1) % 8].state
    #next_3_phase_state = traci.trafficlight.getCompleteRedYellowGreenDefinition(intersection_id)[0].phases[(nowphase_index + 3) % 8].state
    #for edge in Intersection_Edge_Dict[intersection_id]['in']:
    for (index,lane) in enumerate(Global_Vars.LightLib[intersection_id].controlled_lanes):
        if lane  in checked_lane:
            continue
        checked_lane.append(lane)
        if lane[-1]=='0':#不处理右转车道
            continue

        now_signal_state = traffic_signal_dict[current_phase_state[index].lower()]
        next_signal_state = traffic_signal_dict[next_phase_state[index].lower()]
        next_3_signal_state = traffic_signal_dict[next_3_phase_state[index].lower()]
        if now_signal_state == 2:
            vehicle_ids = Global_Vars.JuncLib[intersection_id].vehicle_ids[lane]
            vehicle_occupancy_length = sum(Global_Vars.VehicleLib[vehicle_id].length for vehicle_id in vehicle_ids)
            dentisy_self = vehicle_occupancy_length/Global_Vars.Lanes_length[lane] #self占用率
            #links = traci.lane.getLinks(lane)
            lane_index = lane_index_dict[lane]
            to_list = np.nonzero(lane_adj[lane_index])[0] #这个lane要去的lane的索引
            from_list = np.nonzero(lane_adj[:, lane_index])[0]#来这个lane的索引
            next_state_of_last.extend([dentisy_self])

            # 这里需要做改动，因为以前，一个路口可以查找到进、出这个路口的所有车道
            # 但是现在，一个路口只能查找到进这个路口的车道
            for one_lane in to_list:
                if match_strings(Global_Vars.reversed_lane_dict[str(one_lane)][:-2],lane[:-2]):
                    continue
                one_lane = Global_Vars.reversed_lane_dict[str(one_lane)]
                intersection_to = one_lane.split('t')[1].split('_')[0]
                if intersection_to not in Global_Vars.Intelligent_Sigal_List: #该路口的某条离开车道归属于非智能列表内的路口，则直接认为无堵塞、99秒绿灯
                    next_state_of_last.extend([0,1,99])  
                    continue

                vehicle_ids = Global_Vars.JuncLib[intersection_to].vehicle_ids[one_lane]
                vehicle_occupancy_length = sum(Global_Vars.VehicleLib[vehicle_id].length for vehicle_id in vehicle_ids)
                dentisy_to = vehicle_occupancy_length/Global_Vars.Lanes_length[one_lane]
                signal_index = traffic_signal_dict[Global_Vars.LightLib[intersection_to].phase[one_lane]]#get_lane_state(one_lane, lane_index_dict, lane_adj)
                remain_time = Global_Vars.LightLib[intersection_to].remaining_time[one_lane]
                next_state_of_last.extend([dentisy_to,signal_index,remain_time])

            from_temp = []
            for one_lane in from_list:
                if match_strings(Global_Vars.reversed_lane_dict[str(one_lane)][:-2],lane[:-2]):
                    continue
                one_lane = Global_Vars.reversed_lane_dict[str(one_lane)]
                intersection_from = one_lane.split('t')[1].split('_')[0]
                vehicle_ids = Global_Vars.JuncLib[intersection_from].vehicle_ids[one_lane]
                vehicle_occupancy_length = sum(Global_Vars.VehicleLib[vehicle_id].length for vehicle_id in vehicle_ids)
                dentisy_from = vehicle_occupancy_length/Global_Vars.Lanes_length[one_lane]
                signal_index = traffic_signal_dict[Global_Vars.LightLib[intersection_from].phase[one_lane]]#get_lane_state(one_lane, lane_index_dict, lane_adj)
                remain_time = Global_Vars.LightLib[intersection_from].remaining_time[one_lane]
                from_temp.extend([dentisy_from, signal_index, remain_time])
            from_temp += [0] * (3 * 3 - len(from_temp))
            next_state_of_last.extend(from_temp)

        elif now_signal_state == 0 and next_signal_state == 1:
            vehicle_ids = Global_Vars.JuncLib[intersection_id].vehicle_ids[lane]
            vehicle_occupancy_length = sum(Global_Vars.VehicleLib[vehicle_id].length for vehicle_id in vehicle_ids)
            dentisy_self = vehicle_occupancy_length/Global_Vars.Lanes_length[lane] #self占用率
            #links = traci.lane.getLinks(lane)
            lane_index = lane_index_dict[lane]
            to_list = np.nonzero(lane_adj[lane_index])[0] #这个lane要去的lane的索引
            from_list = np.nonzero(lane_adj[:, lane_index])[0]#来这个lane的索引
            new_state.extend([dentisy_self])
            next_green_density_last.append(dentisy_self)
            for one_lane in to_list:
                if match_strings(Global_Vars.reversed_lane_dict[str(one_lane)][:-2],lane[:-2]):
                    continue
                one_lane = Global_Vars.reversed_lane_dict[str(one_lane)]
                intersection_to = one_lane.split('t')[1].split('_')[0]
                if intersection_to not in Global_Vars.Intelligent_Sigal_List: #该路口的某条离开车道归属于非智能列表内的路口，则直接认为无堵塞、99秒绿灯
                    new_state.extend([0,1,99])  
                    continue
                vehicle_ids = Global_Vars.JuncLib[intersection_to].vehicle_ids[one_lane]
                vehicle_occupancy_length = sum(Global_Vars.VehicleLib[vehicle_id].length for vehicle_id in vehicle_ids)
                dentisy_to = vehicle_occupancy_length/Global_Vars.Lanes_length[one_lane]
                signal_index = traffic_signal_dict[Global_Vars.LightLib[intersection_to].phase[one_lane]]#get_lane_state(one_lane, lane_index_dict, lane_adj)
                remain_time = Global_Vars.LightLib[intersection_to].remaining_time[one_lane]
                new_state.extend([dentisy_to,signal_index,remain_time])

            from_temp = []
            for one_lane in from_list:
                if match_strings(Global_Vars.reversed_lane_dict[str(one_lane)][:-2],lane[:-2]):
                    continue
                one_lane = Global_Vars.reversed_lane_dict[str(one_lane)]
                intersection_from = one_lane.split('t')[1].split('_')[0]
                vehicle_ids = Global_Vars.JuncLib[intersection_from].vehicle_ids[one_lane]
                vehicle_occupancy_length = sum(Global_Vars.VehicleLib[vehicle_id].length for vehicle_id in vehicle_ids)
                dentisy_from = vehicle_occupancy_length / Global_Vars.Lanes_length[one_lane]
                signal_index = traffic_signal_dict[Global_Vars.LightLib[intersection_from].phase[one_lane]]#get_lane_state(one_lane, lane_index_dict, lane_adj)
                remain_time = Global_Vars.LightLib[intersection_from].remaining_time[one_lane]
                from_temp.extend([dentisy_from,signal_index,remain_time])
            from_temp += [0]*(3*3 - len(from_temp))
            new_state.extend(from_temp)


        elif now_signal_state == 0 and next_signal_state == 0 and next_3_signal_state == 1:
            vehicle_ids = Global_Vars.JuncLib[intersection_id].vehicle_ids[lane]
            vehicle_occupancy_length = sum(Global_Vars.VehicleLib[vehicle_id].length for vehicle_id in vehicle_ids)
            dentisy_self = vehicle_occupancy_length / Global_Vars.Lanes_length[lane]  # self占用率
            next_green_density_new.append(dentisy_self)
    next_state_of_last.extend(next_green_density_last)
    new_state.extend(next_green_density_new)

    if len(next_state_of_last) !=40 or len(new_state) != 40:
        print('wrong')


    return np.array(next_state_of_last, dtype=np.float32),np.array(new_state, dtype=np.float32)



def get_reward(intersection_id,agent,Action_list,junction_counts):
    reward = 0
    checked_lane = []
    traffic_signal_dict = {'r': 0, 'g': 1, 'y':2}
    passed_count = junction_counts[intersection_id] - agent.passed_count
    passed_vel = passed_count/Action_list[agent.action]
    agent.passed_count = junction_counts[intersection_id]
    reward = passed_vel*100
    return reward



class QNetwork(nn.Module):
    def __init__(self, state_size, action_size):
        super(QNetwork, self).__init__()
        self.fc1 = nn.Linear(state_size, 256)
        self.fc2 = nn.Linear(256, 128)
        self.fc3 = nn.Linear(128, 64)
        self.fc4 = nn.Linear(64, action_size)

    def forward(self, state):
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))
        x = torch.relu(self.fc3(x))
        return self.fc4(x)


class DDQNAgent:
    def __init__(self, state_size, action_size,device = 'cpu'):
        self.state_size = state_size
        self.action_size = action_size #[10,15,20,25,30,35,40] 7
        self.device = device
        self.memory = deque(maxlen=2000)
        self.gamma = 0.99
        self.epsilon = 0.5
        self.epsilon_decay = 0.99
        self.epsilon_min = 0.01
        self.learning_rate = 0.001
        self.q_network = QNetwork(state_size, action_size).to(self.device)
        self.target_network = QNetwork(state_size, action_size)
        self.target_network.load_state_dict(self.q_network.state_dict())
        #self.optimizer = optim.Adam(self.q_network.parameters(), lr=self.learning_rate)
        self.optimizer = None
        self.update_target_network()
        self.now_Duration = 0
        self.ChangeOrNot = False
        self.CheckOrNot = False
        self.state = np.zeros(state_size, dtype=np.float32)
        self.action = 0
        self.total_reward = 0
        self.reward_delta = 0
        self.step = 0
        self.Trained_time = 0
        self.passed_count = 0


        


    def update_target_network(self):
        self.target_network.load_state_dict(self.q_network.state_dict())

    def act(self, state):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        state = torch.FloatTensor(state).unsqueeze(0)
        act_values = self.q_network(state)
        return torch.argmax(act_values, dim=1).item()

    def train(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)
        self.optimizer = optim.Adam(self.q_network.parameters(), lr=self.learning_rate)
        for state, action, reward, next_state in minibatch:
            target = reward

            next_action = torch.argmax(self.q_network(torch.FloatTensor(next_state).unsqueeze(0)), dim=1)
            target += self.gamma * self.target_network(torch.FloatTensor(next_state).unsqueeze(0))[0][next_action]

            target_f = self.q_network(torch.FloatTensor(state).unsqueeze(0)).detach().clone()
            target_f[0][action] = target

            self.optimizer.zero_grad()
            loss = nn.MSELoss()(self.q_network(torch.FloatTensor(state).unsqueeze(0)), target_f)
            loss.backward()
            self.optimizer.step()
        self.Trained_time += 1

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay







class TrafficLightController(threading.Thread):
    def __init__(self, Traffic_Signal_id_,traffic_light_to_lanes_,lane_index_dict,lane_adj_matrix,N,dt,L_safe,device):
        threading.Thread.__init__(self)
        self.Traffic_Signal_id = Traffic_Signal_id_
        self.device = device
        self.agent = DDQNAgent(state_size,len(Action_list),self.device)
        self.running = True
        self.idm_acc = 0
        self.control_signal = []
        self.time_when_cal = 0
        self.traffic_light_to_lanes = traffic_light_to_lanes_
        self.last_quarter_vehicles = {}
        self.N = N
        self.dt = dt
        self.L_safe = L_safe
        self.lane_index_dict = lane_index_dict
        self.lane_adj_matrix = lane_adj_matrix

        if shared_model:
            try:
                self.agent.q_network.load_state_dict(torch.load(shared_model_location))
                self.agent.target_network.load_state_dict(torch.load(shared_model_location))
            except:
                pass
        else:
            try:
                self.agent.q_network.load_state_dict(torch.load(f'models/{self.Traffic_Signal_id}_model.pth'))
                self.agent.target_network.load_state_dict(torch.load(f'models/{self.Traffic_Signal_id}_model.pth'))
            except:
                pass       


 
    def run(self):

        while self.running > 0:
            if Global_Vars.LightLib[self.Traffic_Signal_id].nowphase_index in [1,3,5,7] and Global_Vars.LightLib[self.Traffic_Signal_id].remaining_time_common < Least_Check_Time and self.agent.CheckOrNot is False:
                next_state,new_state = get_state(self.Traffic_Signal_id,self.lane_index_dict,self.lane_adj_matrix)
                reward = get_reward(self.Traffic_Signal_id,self.agent,Action_list,Global_Vars.junction_counts)
                self.agent.memory.append((self.agent.state, self.agent.action, reward, next_state))
                self.agent.step += 1
                self.agent.action = self.agent.act(next_state)
                self.agent.state = new_state
                # traci.trafficlight.setPhase(self.Traffic_Signal_id, Action_List[action])    设置相位  不明白为什么这里注释了
                # 应该是改用了setPhaseDuration的方法
                self.agent.reward_delta = reward    
                self.agent.total_reward += reward
                self.agent.CheckOrNot = True       



            if Global_Vars.LightLib[self.Traffic_Signal_id].nowphase_index in [0,2,4,6] and self.agent.CheckOrNot is True and Global_Vars.LightLib[self.Traffic_Signal_id].remaining_time_common > Least_Check_Time:
                traci.trafficlight.setPhaseDuration(self.Traffic_Signal_id, float(Action_list[self.agent.action]))
                ###print(f"Agent: {Traffic_Signal_id} 原:{temp_duration} 现在:{get_remaining_phase_time(Traffic_Signal_id)} ")
                self.agent.CheckOrNot = False
                if self.agent.step%train_gap == 0 and self.agent.step>=train_batchsize and Train_Or_Not:
                    self.agent.train(train_batchsize)
                    self.agent.update_target_network()
                    torch.save(self.agent.q_network.state_dict(), f'models/{self.Traffic_Signal_id}_model.pth')
                    ##print(f"Agent: {self.Traffic_Signal_id} Reward = {self.agent.reward_delta} Epsilon = {self.agent.epsilon} Trained_time = {self.agent.Trained_time}")








