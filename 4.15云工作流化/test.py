import redis
r = redis.Redis(host='192.168.100.21', port=6379, db=0)
import msgpack


# 包含二进制数据的字典
data = {
    "name": "example",
    "binary_data": "hello world"
}
class testtarget():
    def __init__(self):
        self.name = "123"
    def show(self):
        print(self.name)

test = testtarget().__dict__
# 使用默认设置（use_bin_type=False）进行序列化
packed_default = msgpack.packb(data)
print(test["name"])
# 使用 use_bin_type=True 进行序列化
packed_with_bin_type = msgpack.packb(data, use_bin_type=True)
r.set("1", packed_with_bin_type)
print(f"Packed with default settings: {packed_default}")
print(f"Packed with use_bin_type=True: {packed_with_bin_type}")

Vehicle_IDs = set([1,2,3,4])
Vehicle_IDs = msgpack.packb(list(Vehicle_IDs),use_bin_type = True)
r.set("Vehicle_IDs", Vehicle_IDs)
Vehicle_IDs = set(msgpack.unpackb(r.get("Vehicle_IDs"), raw=False))
print(Vehicle_IDs)


key_type = r.type("JuncLib").decode('utf-8')
print(f"键 'JuncLib' 的类型是: {key_type}")
JuncLib_dict = r.hgetall("JuncLib")
JuncLib = {}
for key in JuncLib_dict.keys():
    key_str = key.decode()
    print(msgpack.unpackb(JuncLib_dict[key], raw=False))

key_type = r.type("LightLib").decode('utf-8')
print(f"键 'LightLib' 的类型是: {key_type}")
LightLib_dict = r.get("LightLib")
LightLib_dict = msgpack.unpackb(LightLib_dict, raw=False)
print(LightLib_dict)
print(type(LightLib_dict))

Vehicle_Lib = {}
Vehicle_Lib = msgpack.unpackb(r.get("VehicleLib"), raw=False)
print(Vehicle_Lib)
