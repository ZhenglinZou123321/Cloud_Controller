import traci
import threading
import time
import copy
import numpy as np
from Solver_utils import *
import torch
import torch.nn as nn
import random



state_size = 1+3*3+3*3+1+3*3+3*3+2
Action_list = [10,15,20,25,30,35,40,45,50,55,60,65,70,75,80]

shared_model = True
shared_model_location = 'models/agent_model.pth'
Train_Or_Not = False

Least_Check_Time = 3

train_gap = 20
train_batchsize = 32

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


# 合并所有智能体的 memory
def merge_and_save_memory(agent_list, file_path):
    merged_memory = []

    for agent in agent_list.values():  # 遍历每个智能体
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

def is_incoming_lane(lane_id):
    # 获取该车道的链接信息
    links = traci.lane.getLinks(lane_id)
    if not links:
        return False  #如果没有链接，则不属于驶入车道

    # 如果第一段链接是进入路口的（交叉口），则该车道为驶入车道
    next_lane_id, via_edge_id, signal_index, traffic_light_id = links[0]
    if traffic_light_id:
        return True  #有信号灯控制则为驶入路口车道
    return False

def get_remaining_phase_time(traffic_light_id): #获取信号灯剩余时间
    # 获取当前仿真时间
    current_time = traci.simulation.getTime()
    # 获取下一个信号切换的时间
    next_switch_time = traci.trafficlight.getNextSwitch(traffic_light_id)
    # 计算剩余时间
    remaining_time = next_switch_time - current_time
    return max(remaining_time, 0)  # 防止负值

def get_lane_state(lane_id,lane_dict,lane_m):
    traffic_signal_dict = {'r':0,'g':1,'y':2}
    edge_id = traci.lane.getEdgeID(lane_id)
    to_junction = traci.edge.getToJunction(edge_id)
    if to_junction in traffic_light_to_lanes.keys():
        controlled_lanes = traci.trafficlight.getControlledLanes(to_junction)
        current_phase_state = traci.trafficlight.getRedYellowGreenState(to_junction)
        lane_index = controlled_lanes.index(lane_id)
        lane_phase = current_phase_state[lane_index]
        remain_time = get_remaining_phase_time(to_junction)
        return traffic_signal_dict[lane_phase.lower()], remain_time
    else:
        lane_phase = 'g'
        remain_time = 99

    return traffic_signal_dict[lane_phase],remain_time


def get_state(intersection_id,Intersection_Edge_Dict,lane_index_dict,lane_adj,nowphase_index):
    reversed_lane_dict = {str(v): k for k, v in lane_index_dict.items()}
    next_state_of_last = []
    new_state = []
    traffic_signal_dict = {'r':0,'g':1,'y':2}
    checked_lane = []
    dentisy_self = 0#不处理右转车道
    dentisy_from = 0
    dentisy_to = 0#目标车道的车辆密度
    next_green_density_last = []
    next_green_density_new = []
    current_phase_state = traci.trafficlight.getRedYellowGreenState(intersection_id)
    next_phase_state = traci.trafficlight.getCompleteRedYellowGreenDefinition(intersection_id)[0].phases[(nowphase_index + 1) % 8].state
    next_3_phase_state = traci.trafficlight.getCompleteRedYellowGreenDefinition(intersection_id)[0].phases[(nowphase_index + 3) % 8].state
    #for edge in Intersection_Edge_Dict[intersection_id]['in']:
    for (index,lane) in enumerate(traci.trafficlight.getControlledLanes(intersection_id)):
        if lane  in checked_lane:
            continue
        checked_lane.append(lane)
        if lane[-1]=='0':#不处理右转车道
            continue

        now_signal_state = traffic_signal_dict[current_phase_state[index].lower()]
        next_signal_state = traffic_signal_dict[next_phase_state[index].lower()]
        next_3_signal_state = traffic_signal_dict[next_3_phase_state[index].lower()]
        if now_signal_state == 2:
            vehicle_ids = traci.lane.getLastStepVehicleIDs(lane)
            vehicle_occupancy_length = sum(traci.vehicle.getLength(vehicle_id) for vehicle_id in vehicle_ids)
            dentisy_self = vehicle_occupancy_length/traci.lane.getLength(lane) #self占用率
            #links = traci.lane.getLinks(lane)
            lane_index = lane_index_dict[lane]
            to_list = np.nonzero(lane_adj[lane_index])[0] #这个lane要去的lane的索引
            from_list = np.nonzero(lane_adj[:, lane_index])[0]#来这个lane的索引
            next_state_of_last.extend([dentisy_self])
            for one_lane in to_list:
                if match_strings(reversed_lane_dict[str(one_lane)][:-2],lane[:-2]):
                    continue
                one_lane = reversed_lane_dict[str(one_lane)]
                vehicle_ids = traci.lane.getLastStepVehicleIDs(one_lane)
                vehicle_occupancy_length = sum(traci.vehicle.getLength(vehicle_id) for vehicle_id in vehicle_ids)
                dentisy_to = vehicle_occupancy_length/traci.lane.getLength(one_lane)
                signal_index,remain_time = get_lane_state(one_lane, lane_index_dict, lane_adj)
                next_state_of_last.extend([dentisy_to,signal_index,remain_time])

            from_temp = []
            for one_lane in from_list:
                if match_strings(reversed_lane_dict[str(one_lane)][:-2],lane[:-2]):
                    continue
                one_lane = reversed_lane_dict[str(one_lane)]
                vehicle_ids = traci.lane.getLastStepVehicleIDs(one_lane)
                vehicle_occupancy_length = sum(traci.vehicle.getLength(vehicle_id) for vehicle_id in vehicle_ids)
                dentisy_from = vehicle_occupancy_length/traci.lane.getLength(one_lane)
                signal_index,remain_time = get_lane_state(one_lane, lane_index_dict, lane_adj)
                from_temp.extend([dentisy_to, signal_index, remain_time])
            from_temp += [0] * (3 * 3 - len(from_temp))
            next_state_of_last.extend(from_temp)

        elif now_signal_state == 0 and next_signal_state == 1:
            vehicle_ids = traci.lane.getLastStepVehicleIDs(lane)
            vehicle_occupancy_length = sum(traci.vehicle.getLength(vehicle_id) for vehicle_id in vehicle_ids)
            dentisy_self = vehicle_occupancy_length/traci.lane.getLength(lane) #self占用率
            #links = traci.lane.getLinks(lane)
            lane_index = lane_index_dict[lane]
            to_list = np.nonzero(lane_adj[lane_index])[0] #这个lane要去的lane的索引
            from_list = np.nonzero(lane_adj[:, lane_index])[0]#来这个lane的索引
            new_state.extend([dentisy_self])
            next_green_density_last.append(dentisy_self)
            for one_lane in to_list:
                if match_strings(reversed_lane_dict[str(one_lane)][:-2],lane[:-2]):
                    continue
                one_lane = reversed_lane_dict[str(one_lane)]
                vehicle_ids = traci.lane.getLastStepVehicleIDs(one_lane)
                vehicle_occupancy_length = sum(traci.vehicle.getLength(vehicle_id) for vehicle_id in vehicle_ids)
                dentisy_to = vehicle_occupancy_length/traci.lane.getLength(one_lane)
                signal_index,remain_time = get_lane_state(one_lane, lane_index_dict, lane_adj)
                new_state.extend([dentisy_to,signal_index,remain_time])

            from_temp = []
            for one_lane in from_list:
                if match_strings(reversed_lane_dict[str(one_lane)][:-2],lane[:-2]):
                    continue
                one_lane = reversed_lane_dict[str(one_lane)]
                vehicle_ids = traci.lane.getLastStepVehicleIDs(one_lane)
                vehicle_occupancy_length = sum(traci.vehicle.getLength(vehicle_id) for vehicle_id in vehicle_ids)
                dentisy_from = vehicle_occupancy_length/traci.lane.getLength(one_lane)
                signal_index,remain_time = get_lane_state(one_lane, lane_index_dict, lane_adj)
                from_temp.extend([dentisy_to,signal_index,remain_time])
            from_temp += [0]*(3*3 - len(from_temp))
            new_state.extend(from_temp)


        elif now_signal_state == 0 and next_signal_state == 0 and next_3_signal_state == 1:
            vehicle_ids = traci.lane.getLastStepVehicleIDs(lane)
            vehicle_occupancy_length = sum(traci.vehicle.getLength(vehicle_id) for vehicle_id in vehicle_ids)
            dentisy_self = vehicle_occupancy_length / traci.lane.getLength(lane)  # self占用率
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
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size #[10,15,20,25,30,35,40] 7
        self.memory = deque(maxlen=2000)
        self.gamma = 0.99
        self.epsilon = 0.5
        self.epsilon_decay = 0.99
        self.epsilon_min = 0.01
        self.learning_rate = 0.001
        self.q_network = QNetwork(state_size, action_size)
        self.target_network = QNetwork(state_size, action_size)
        self.optimizer = optim.Adam(self.q_network.parameters(), lr=self.learning_rate)
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
    def __init__(self, Traffic_Signal_id_,traffic_light_to_lanes_,lane_index_dict,lane_adj_matrix,N,dt,L_safe):
        threading.Thread.__init__(self)
        self.Traffic_Signal_id = Traffic_Signal_id_
        self.agent = DDQNAgent(state_size,len(Action_list))
        self.running = True
        self.idm_acc = 0
        self.control_signal = []
        self.time_when_cal = 0
        self.traffic_light_to_lanes = traffic_light_to_lanes_
        self.last_quarter_vehicles = {}
        self.N = N
        self.dt = dt
        self.L_safe = L_safe
        self.Intersection_Edge_Dict = Intersection_Edge_Dict
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
        global step
        global junction_counts
        while self.running and traci.simulation.getMinExpectedNumber() > 0:
            if traci.trafficlight.getPhase(self.Traffic_Signal_id) in [1,3,5,7] and get_remaining_phase_time(self.Traffic_Signal_id)<Least_Check_Time and self.agent.CheckOrNot is False:
                next_state,new_state = get_state(self.Traffic_Signal_id,self.Intersection_Edge_Dict,self.lane_index_dict,self.lane_adj_matrix,traci.trafficlight.getPhase(self.Traffic_Signal_id))
                reward = get_reward(self.Traffic_Signal_id,self.agent,Action_list,junction_counts)
                self.agent.memory.append((self.agent.state, self.agent.action, reward, next_state))
                self.agent.step += 1
                self.agent.action = self.agent.act(next_state)
                self.agent.state = new_state
                #traci.trafficlight.setPhase(self.Traffic_Signal_id, Action_List[action])    设置相位  不明白为什么这里注释了
                # 应该是改用了setPhaseDuration的方法
                self.agent.reward_delta = reward
                self.agent.total_reward += reward
                self.agent.CheckOrNot = True       

        if traci.trafficlight.getPhase(self.Traffic_Signal_id) in [0,2,4,6] and self.agent.CheckOrNot is True and get_remaining_phase_time(self.Traffic_Signal_id)>Least_Check_Time:
            temp_duration = get_remaining_phase_time(self.Traffic_Signal_id)
            traci.trafficlight.setPhaseDuration(self.Traffic_Signal_id, float(Action_list[self.agent.action]))
            #print(f"Agent: {Traffic_Signal_id} 原:{temp_duration} 现在:{get_remaining_phase_time(Traffic_Signal_id)} ")
            self.agent.CheckOrNot = False
            if self.agent.step%train_gap == 0 and self.agent.step>=train_batchsize and Train_Or_Not:
                self.agent.train(train_batchsize)
                self.agent.update_target_network()
                torch.save(self.agent.q_network.state_dict(), f'models/{self.Traffic_Signal_id}_model.pth')
                print(f"Agent: {self.Traffic_Signal_id} Reward = {self.agent.reward_delta} Epsilon = {self.agent.epsilon} Trained_time = {self.agent.Trained_time}")








