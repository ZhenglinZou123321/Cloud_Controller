import Global_Vars
from utils.junction_utils import *
import msgpack
import redis
r = redis.Redis(host='192.168.100.21', port=6379, db=0)

def junction_run(id):
    id = id
    juncclass = Global_Vars.Junc(id)
    Global_Vars.JuncLib[id] = juncclass
    junc_controller = JunctionController(id,Global_Vars.traffic_light_to_lanes,Global_Vars.N,Global_Vars.dt,Global_Vars.L_safe)
    Global_Vars.junction_threads[id]= junc_controller
    while traci.simulation.getMinExpectedNumber() > 0:
        try:
            Global_Vars.Vehicle_IDs = msgpack.unpackb(r.get("Vehicle_IDs"), raw=False)   
        except: 
            print("未能从redis读取Vehicle_IDs")

        try:
            Global_Vars.VehicleLib = msgpack.unpackb(r.get("VehicleLib"), raw=False)   
        except: 
            print("未能从redis读取VehicleLib")

        try:
            Global_Vars.JuncLib = r.get("JuncLib")
            Global_Vars.JuncLib = msgpack.unpackb(Global_Vars.JuncLib, raw=False)

        except:
            print("未能从redis读取JuncLib")
        
        try:
            Global_Vars.LightLib = r.get("LightLib")
            Global_Vars.LightLib = msgpack.unpackb(Global_Vars.LightLib, raw=False)
        except:
            print("未能从redis读取LightLib")

        try:
            Global_Vars.simulate_info = r.get("simulate_info")
            Global_Vars.simulate_info = msgpack.unpackb(Global_Vars.simulate_info, raw=False)
        except:
            print("未能从redis读取simulate_info")

        Global_Vars.JuncLib[id].update()
        Global_Vars.JuncLib[id].Vehicle_Control()  
        if Global_Vars.step % 5 == 0:
            junc_controller.run()
        
        Global_Vars.JuncLib[id] = msgpack.packb(Global_Vars.JuncLib[id], use_bin_type=True)
        r.hset("JuncLib",id,Global_Vars.JuncLib[id])

        r.hset("Vehicle_IDs",msgpack.packb(Global_Vars.Vehicle_IDs, use_bin_type=True))

        traci.simulationStep()


