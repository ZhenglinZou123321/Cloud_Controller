import Global_Vars
from utils.junction_utils import *

def junction_run(id):
    id = id
    juncclass = Global_Vars.Junc(id)
    Global_Vars.JuncLib[id] = juncclass
    junc_controller = JunctionController(id,Global_Vars.traffic_light_to_lanes,Global_Vars.N,Global_Vars.dt,Global_Vars.L_safe)
    Global_Vars.junction_threads[id]= junc_controller
    while traci.simulation.getMinExpectedNumber() > 0:
        Global_Vars.JuncLib[id].update()
        Global_Vars.JuncLib[id].Vehicle_Control()  
        if Global_Vars.step % 5 == 0:
            junc_controller.run()
        
        traci.simulationStep()


