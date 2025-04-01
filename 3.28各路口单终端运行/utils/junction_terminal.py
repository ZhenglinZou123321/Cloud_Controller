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
            VehicleLib_dict = msgpack.unpackb(r.get("VehicleLib"), raw=False)
            for key in VehicleLib_dict.keys():
                Global_Vars.VehicleLib[key].Conv_from_dict(VehicleLib_dict[key])
        except: 
            print("未能从redis读取VehicleLib")

        try:
            JuncLib_dict = msgpack.unpackb(r.get("JuncLib"), raw=False)
            for key in JuncLib_dict.keys():
                Global_Vars.JuncLib[key].Conv_from_dict(JuncLib_dict[key])
        except:
            print("未能从redis读取JuncLib")
        
        try: 
            LightLib_dict = msgpack.unpackb(r.get("LightLib"), raw=False)
            for key in LightLib_dict.keys():
                Global_Vars.LightLib[key].Conv_from_dict(LightLib_dict[key])
        except:
            print("未能从redis读取LightLib")

        try:
            Global_Vars.simulate_info.Conv_from_dict(msgpack.unpackb(r.get("simulate_info"), raw=False))
        except:
            print("未能从redis读取simulate_info")

        Global_Vars.JuncLib[id].update()
        Global_Vars.JuncLib[id].Vehicle_Control()  
        if Global_Vars.step % 5 == 0:
            junc_controller.run()
        
        JuncLib_dict[id] = msgpack.packb(Global_Vars.JuncLib[id].__dict__, use_bin_type=True)
        r.hset("JuncLib",id,JuncLib_dict[id])

        r.hset("Vehicle_IDs",msgpack.packb(Global_Vars.Vehicle_IDs, use_bin_type=True))

        traci.simulationStep()


