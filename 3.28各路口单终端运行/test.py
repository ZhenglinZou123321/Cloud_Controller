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
