import Global_Vars
from utils.junction_utils import *
import msgpack
import redis
r = redis.Redis(host='192.168.100.21', port=6379, db=0)

def junction_run(id,task1_ip,task1_port,Control_Or_Not=False):
    for junc in Global_Vars.Intelligent_Sigal_List:
        lightclass = Global_Vars.Light(junc)
        juncclass = Global_Vars.Junc(junc)
        Global_Vars.LightLib[junc] = lightclass
        Global_Vars.JuncLib[junc] = juncclass
    id = id
    junc_controller = JunctionController(id,Global_Vars.traffic_light_to_lanes,Global_Vars.N,Global_Vars.dt,Global_Vars.L_safe,redis_base=r,task1_ip=task1_ip,task1_port=task1_port)
    Global_Vars.junction_threads[id]= junc_controller
    JuncLib_dict = {}
    while traci.simulation.getMinExpectedNumber() > 0:
        try:
            Global_Vars.Vehicle_IDs = set(msgpack.unpackb(r.get("Vehicle_IDs"), raw=False))   
        except: 
            print("未能从redis读取Vehicle_IDs")

        try:
            VehicleLib_dict = msgpack.unpackb(r.get("VehicleLib"), raw=False)
            for key in VehicleLib_dict.keys():
                if key not in Global_Vars.VehicleLib.keys():
                    Global_Vars.VehicleLib[key] = Global_Vars.Vehicle(key,key[0:3])
                Global_Vars.VehicleLib[key].Conv_from_dict(VehicleLib_dict[key])
        except: 
            print("未能从redis读取VehicleLib")
        

        try:
            JuncLib_dict = r.hgetall("JuncLib")
            for key in JuncLib_dict.keys():
                try:
                    # 将键从字节类型解码为字符串类型
                    key_str = key.decode()
                    try:
                        # 对哈希表中键对应的值进行 msgpack 反序列化
                        unpacked_value = msgpack.unpackb(JuncLib_dict[key], raw=False)
                        # 调用对象的方法将反序列化后的数据进行转换
                        Global_Vars.JuncLib[key_str].Conv_from_dict(unpacked_value)
                    except msgpack.UnpackException as e:
                        print(f"对键 {key_str} 对应的值进行 msgpack 反序列化时出错: {e}")
                    except AttributeError as e:
                        print(f"在调用 Global_Vars.JuncLib[{key_str}].Conv_from_dict 方法时出错: {e}")
                    except KeyError as e:
                        print(f"Global_Vars.JuncLib 中不存在键 {key_str}: {e}")
                except UnicodeDecodeError as e:
                    print(f"对键 {key} 进行解码时出错: {e}")
        except redis.exceptions.RedisError as e:
            print(f"从 Redis 获取 JuncLib 时出现 Redis 错误: {e}")
        except Exception as e:
            print(f"出现未知错误: {e}")


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
        
        if Control_Or_Not == True:
            Global_Vars.JuncLib[id].Vehicle_Control() 

        if Global_Vars.step % 20 == 0 and Control_Or_Not == True: #每4s算8s
            start_time = time.time()
            junc_controller.run(Global_Vars.step)
            print(f'控制量计算耗时{time.time()-start_time}')
        
        JuncLib_dict[id] = msgpack.packb(Global_Vars.JuncLib[id].Conv_to_dict() , use_bin_type=True)
        r.hset("JuncLib",id,JuncLib_dict[id])

        r.set("Vehicle_IDs",msgpack.packb(list(Global_Vars.Vehicle_IDs), use_bin_type=True))

        Global_Vars.step += 1
        start_time = time.time()
        traci.simulationStep()
        print(f'traci_simulationStep耗时{time.time()-start_time}')
        print(Global_Vars.step)


