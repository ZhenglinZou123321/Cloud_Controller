import traci

#controller_1 = traci.connect(host="192.168.100.104", port=14491)
traci.init(port = 14491,host="192.168.100.104")
print('111')
#controller_1.setOrder(1) #设置优先级，数字越大，越高
traci.setOrder(1)
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()

    print(traci.vehicle.getIDList())
    continue