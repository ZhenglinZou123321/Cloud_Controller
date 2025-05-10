import traci
# Traci远程连接的设置
traci.init(port = 14491,host="192.168.100.103")
traci.setOrder(5) #设置优先级，数字越小，越高
from utils.junction_terminal import *
if __name__ == "__main__":
    id = 'j5'
    junction_run(id,'192.168.100.34',7005)


