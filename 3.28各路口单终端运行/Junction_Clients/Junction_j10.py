import traci
# Traci远程连接的设置
traci.init(port = 14491,host="192.168.100.104")
traci.setOrder(3) #设置优先级，数字越小，越高
import Global_Vars


if __name__ == "__main__":
    id = 'j10'
    juncclass = Global_Vars.Junc(id)
    junc_controller = Global_Vars.JunctionController(id,Global_Vars.traffic_light_to_lanes,Global_Vars.N,Global_Vars.dt,Global_Vars.L_safe)
    while traci.simulation.getMinExpectedNumber() > 0:
        Global_Vars.JuncLib[id].update()  
        if Global_Vars.step % 5 == 0:
            junc_controller.run()
        
        traci.simulationStep()


