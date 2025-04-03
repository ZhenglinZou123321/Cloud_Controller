import pandas as pd

# 读取 CSV 文件
file_path = 'traffic_data_gaussian.csv'  # 替换为你的 CSV 文件路径
df = pd.read_csv(file_path)

# 按 road_id 进行分组
grouped = df.groupby('road_id')

# 存储每个 road_id 对应的 time 值列表
road_id_time_dict = {}
for road_id, group in grouped:
    road_id_time_dict[road_id] = {'time':group['time'],'vehicle_count':group['vehicle_count'],'average_speed':group['average_speed']}

