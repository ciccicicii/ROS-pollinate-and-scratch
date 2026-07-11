#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import os
import time
from geometry_msgs.msg import Twist, Vector3
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
#import geometry_msgs/PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
#from xf_mic_asr_offline.msg import lty_arrive
#from xf_mic_asr_offline.msg import lty_action
from nav_msgs.msg import Odometry
import serial
#import re
import rospy
#import sys
import math
import string
from sensor_msgs.msg import LaserScan

import actionlib
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import  Float64
import subprocess
import numpy as np
from rospy.numpy_msg  import numpy_msg


from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import tf.transformations  # 添加这一行
from geometry_msgs.msg import Quaternion


import cv2
from cv2 import aruco
from pyzbar.pyzbar import decode
import sys
from std_msgs.msg import Int32MultiArray
fruit_map  = {
            "苹果": 0, "坏苹果": 1,
            "梨子": 2, "坏梨子": 3,
            "番茄": 4, "坏番茄": 5,
            "洋葱": 6, "坏洋葱": 7,
            "辣椒": 8, "坏辣椒": 9,
            "南瓜": 10, "坏南瓜": 11
        }
rospy.init_node('QR_detect',anonymous=True)
pub  = rospy.Publisher('/fruit_array', Int32MultiArray, queue_size=10)
# 确保输出编码为utf-8
if sys.version_info < (3, 0):
    # 在Python 2中，设置标准输出的编码
    reload(sys)  # 重新加载sys模块
    sys.setdefaultencoding('utf-8')
else:
    # Python 3中使用原代码
    if sys.stdout.encoding != 'utf-8':
        sys.stdout = open(sys.stdout.fileno(), mode='w', encoding='utf-8', buffering=1)
def draw_text_with_opencv(frame, text, position):
    """使用OpenCV原生方法绘制文本"""
    cv2.putText(frame, text, position,
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
    return frame
def read_qr_code(frame):
    # 转换为灰度图
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 对比度增强 (CLAHE)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    enhanced_gray = clahe.apply(gray)

    thresh = cv2.adaptiveThreshold(
        gray, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, 11, 2
    )
    '''


    # 方法2: OTSU阈值
    _, methods['otsu'] = cv2.threshold(enhanced_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # 方法3: 锐化 + 自适应阈值
    kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
    sharpened = cv2.filter2D(enhanced_gray, -1, kernel)
    sharpened_blurred = cv2.GaussianBlur(sharpened, (3, 3), 0)
    methods['sharpened'] = cv2.adaptiveThreshold(
        sharpened_blurred, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, 11, 2
    )
    '''

    decoded_objects = decode(thresh)

    # 在原图上绘制结果
    for obj in decoded_objects:
        try:
            qr_content = obj.data.decode('utf-8', errors='replace')
            print("解码内容: {}".format(qr_content))
            content = qr_content.split('\n')
            converted_data = []
            # 处理二维码内容
            if ',' not in qr_content:

                # 纯文本处理

                for item in content:
                    item_clean = str(item).strip()
                    if item_clean in fruit_map:
                        converted_data.append(fruit_map[item_clean])
                    else:
                        print("无效水果")
            else:
                if len(content) > 1:
                    fruit_part = content[:-1]
                    for item in fruit_part:
                        item_clean = str(item).strip()
                        if item_clean in fruit_map:
                            converted_data.append(fruit_map[item_clean])
                        else:
                            print("无效水果")
                # 处理数字部分
                if len(content) >= 1:
                    number_str = content[-1].split(',')
                    for num in number_str:
                        try:
                            converted_data.append(int(num.strip()))
                        except ValueError:
                            print("无效数字")

            # 发布消息
            msg_out = Int32MultiArray()
            msg_out.data  = converted_data
            pub.publish(msg_out)
            # 绘制边框（线条宽度1）
            points = obj.polygon
            if len(points) == 4:
                pts = [(int(point.x), int(point.y)) for point in points]
                for j in range(4):
                    cv2.line(frame, pts[j], pts[(j + 1) % 4], (0, 255, 0), 1)

            # 绘制文本（字体大小0.6，线条宽度1）
            text_x = obj.rect.left
            text_y = max(10, obj.rect.top - 5)
            frame = draw_text_with_opencv(frame, qr_content, (text_x, text_y))
        except Exception as e:
            print("处理二维码时出错: {}".format(e))
            cv2.putText(frame, "ERR", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    return frame

def QR_detect():

    # 获取摄像头图片
    cap = cv2.VideoCapture(3, cv2.CAP_V4L2)
    # 设置摄像头分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    # 设置目标帧率
    target_fps = 30
    frame_delay = 33  # 每帧处理时间（毫秒）
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    try:
        cv2.namedWindow('Camera', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Camera', 640, 480)  # 显示窗口可以更大

        frame_skip = 2
        frame_count = 0
        frame_times = []

        while True:
            start_time = time.time()

            ret, frame = cap.read()
            if not ret:
                print("无法获取帧")
                break

            frame_count += 1
            if frame_count % frame_skip != 0:
                # 跳过处理，直接显示
                cv2.imshow('Camera', frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
                continue

            # 仅处理关键帧
            processed_frame = read_qr_code(frame)
            cv2.imshow('Camera', processed_frame)

            if cv2.waitKey(1) & 0xFF == 27:  # 按ESC键退出
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__=="__main__":
    try:
        rospy.loginfo('QR_detect node started')
        QR_detect()
        # 从终端读取键值
        key = sys.stdin.read(1)
        if key == 'q':
            sys.exit(" 程序终止")  # 可传递退出码或字符串
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")
