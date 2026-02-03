#负责打开摄像机 拍照 发送 ArUco 码id加四个角的坐标到 ZMQ 网络广播
#发送格式：id x0 y0 x1 y1 x2 y2 x3 y3
#拍照频率可调，默认30FPS（30 次/秒）
#发送信息的频率基本和拍照频率一致，拍了多少发多少情况 
# 比如画面里有 3 个码
# 每秒 30 帧
# 每帧发 3 条
# 就是≈ 90 条 / 秒

import time
import zmq# 用于把数据通过网络发送出去
import cv2
import os

# =====================
# ZMQ PUB config
# =====================
PUB_IP = "0.0.0.0" # 表示监听本机所有网络接口，这样局域网内的其他电脑也能连进来
PUB_PORT = 5556  # 和 utils.zmqPublisherPort 对齐 像一个房间号，接收方也得开这个房间号才能领到数据
# 1280*720
# =====================
# Camera config
# =====================
CAM_INDEX = 0  # 默认第一个摄像头
FPS_LIMIT = 30

def main():
    # ZMQ PUB
    ctx = zmq.Context.instance() # 创建一个ZMQ管理中心
    pub = ctx.socket(zmq.PUB) # 声明这是一个“发布者”模式的套接字
    pub.bind(f"tcp://{PUB_IP}:{PUB_PORT}") # 绑定地址和端口，正式开启广播
    print(f"[PUB] bound on tcp://{PUB_IP}:{PUB_PORT}")

    # Camera
    cap = cv2.VideoCapture(CAM_INDEX) # 打开摄像头
    if not cap.isOpened():
        raise RuntimeError("Camera open failed.")
    # # 强制设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, FPS_LIMIT)
    # 打印摄像头实际分辨率
    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("First frame read failed.")
    h, w = frame.shape[:2]
    print(f"Camera confirmed: {w} x {h}", flush=True)

    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    params = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(dictionary, params)

    last = 0.0
    while True:
        ret, frame = cap.read() # 从摄像头里抓一张照片
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # 变灰度图，因为找码不需要颜色，灰色更快
        corners_list, ids, _ = detector.detectMarkers(gray) # 开始找码！

        if ids is not None:
            # 一次性画所有检测到的 ArUco
            cv2.aruco.drawDetectedMarkers(frame, corners_list, ids)

            # 再逐个发 ZMQ
            for corners, tag_id in zip(corners_list, ids.flatten()):
                c = corners[0]
                msg = f"{int(tag_id)} {c[0][0]} {c[0][1]} {c[1][0]} {c[1][1]} {c[2][0]} {c[2][1]} {c[3][0]} {c[3][1]}"
                pub.send_string(msg)


        cv2.imshow("vision_pub", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break
        # 简单限速 如果运行太快，就让程序强制睡一会儿，保证不超过设定的 FPS
        now = time.time()
        dt = now - last
        if dt < 1.0 / FPS_LIMIT:
            time.sleep(max(0.0, 1.0 / FPS_LIMIT - dt))
        last = time.time()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
