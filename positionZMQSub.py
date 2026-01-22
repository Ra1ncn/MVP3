import sys
import zmq
import time
import threading
from threading import Thread
import copy

import utils

# Dictionary
carPosiDict = dict() # 创建一个字典，用来存：{小车ID: 坐标字符串}
#字典 carPosiDict 里面存的是 “小车编号” 到 “四个角点坐标” 的映射。

# Create a threading lock for safe access to dictionary
lock = threading.Lock()
import traceback
print("initialize_zmq called\n", "".join(traceback.format_stack(limit=6)))
#一直在后台线程里拉取 ZMQ 数据，更新 carPosiDict 字典
def _pull_zmq_data():
    #Connect to zmq publisher 
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket # 订阅所有消息

    print(f"Collecting update from server: tcp://{utils.zmqPublisherIP}:{utils.zmqPublisherPort}")
    socket.connect ("tcp://%s:%s" %(utils.zmqPublisherIP,
        utils.zmqPublisherPort))
    print ("Connected...")
    socket.setsockopt(zmq.SUBSCRIBE, b"")

    #Continuous update of car position, if available
    while True:     # 接收原始字符串，如 "1 100 200..."
        msg = socket.recv_string()          # 直接拿到 str
        carID_str, rest = msg.split(" ", 1) # 拆成 "id" 和 "后面整串坐标"
        with lock:
            carPosiDict[int(carID_str)] = rest


# function to sense the tag of all cars at very beginning 
# 这个函数通常只在程序刚启动时跑一次，用来确定场上一共有几辆车。
#执行这个函数 会：
# 1修改全局列表：它会把识别到的所有小车 ID 存入 utils.carInfo 这个列表里。
# 2完成“初始化扫场”：确保程序知道当前场地上一共有哪些标记。
def _pull_zmq_data_once():
    #Connect to zmq publisher 
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket

    print(f"Collecting update from server: tcp://{utils.zmqPublisherIP}:{utils.zmqPublisherPort}")
    socket.connect ("tcp://%s:%s" %(utils.zmqPublisherIP,
        utils.zmqPublisherPort))
    print ("Connected...")
    socket.setsockopt(zmq.SUBSCRIBE, b"")

    #Continuous update of car position, if available
    flag = True
    count = 0
    while flag:
        msg = socket.recv_string()              # 直接拿到一整行字符串
        carID_str, rest = msg.split(" ", 1)     # 只切一次：前面是id，后面是坐标串
        carIDI = int(carID_str)

        with lock:
            if (carIDI, carIDI) not in utils.carInfo:
                utils.carInfo.append((carIDI, carIDI))
            else:
                count += 1

        if count == 10:
            flag = False

    # print utils.carInfo
    socket.close()
        
#拿到所有小车的位置数据的副本
def _get_all_car_position_data():
    with lock:
        tempData = copy.deepcopy(carPosiDict)
    return tempData
        
#拿到指定小车的位置数据
def _get_car_position_data(carID):
    tempData = ""
    with lock:
        if carPosiDict.has_key(carID):
            tempData = carPosiDict[carID]
    return tempData

_t = None
_zmq_started = False

def _initialize_zmq():
    global _t, _zmq_started
    if _zmq_started:
        return  # 已经启动过就直接返回
    _zmq_started = True

    _t = Thread(target=_pull_zmq_data, daemon=True)
    _t.start()
    time.sleep(0.3)


def _stop_zmq():
    return
