#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import time
from std_msgs.msg import Int32MultiArray
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
import subprocess
from std_msgs.msg import  Float64
import rospy
import json
import sys, select, termios, tty

# 关节角度初始值（单位：度）
pose = [0, 0, 0, 0, 0]  # [base, shoulder, elbow, wrist, gripper]

# 每次按键变化的步长（度）
step_angle = 2.5


#坐标映射函数
def pixel_to_world(u, v, size):
    # 中心点像素
    cx, cy = self.cam_cx, self.cam_cy

    # 像素坐标偏移量
    dx = u - cx
    dy = v - cy

    # 根据目标像素大小估算距离Z（映射表）
    Z = self.size_to_distance(size)

    # 偏移像素到偏移物理距离
    X = dx * self.pixel_scale_x * Z
    Y = dy * self.pixel_scale_y * Z

    return dx, dy, Z

def to_joint(dx,dy):
    # 当前角度（你需要维护一个 pose 变量）
    b = pose[0]
    s = pose[1]

    # 增量调整
    b_delta = -dx * self.k1  # 左偏则右转
    s_delta = dy * self.k2   # 上偏则低头

    # 更新角度
    pose[0] += b_delta
    pose[1] += s_delta

    # 限制关节范围
    pose[0] = max(min(new_pose[0], 120), -120)
    pose[1] = max(min(new_pose[1], 90), -90)
    arm.send_pose(pose)  # 发送新姿态
# 获取键盘按键
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class ArmTeleop:
    def __init__(self, serial_port="/dev/ttyUSB3", baud=115200):
        import serial
        self.serial = serial.Serial(serial_port, baud, timeout=0.01)
        self.M2_speed = 80  # 速度可调
        self.M2_acc = 40    # 加速度可调
        self.cam_cx, self.cam_cy= 320, 240  # 相机中心像素坐标
        self.k1=0.2 #关节偏移角度比例
        self.k2=0.2
        # 订阅视觉数据
        rospy.Subscriber('/yolov5_sign', Int32MultiArray, self.vision_callback)
        self.vision_data = None


        self.XYZT_control = 104
        self.P = [312.232, 6.227, 231.807, 3.14]

    def vision_callback(self, msg):
        # 回调函数，接受视觉数据并存储
        self.vision_data = msg.data  # 数据格式为 [flower_id, x_center, y_center, width, height]
    def send_xyz(self, x, y, z):
        self.P[0] = x
        self.P[1] = y
        self.P[2] = z
        self.P[3] = 3.14                     #104代表三位坐标目标点控制方式，x y z是坐标，t是末端角度，spd是速度
        self.serial.write((json.dumps({"T":104, \
                                       "x":self.P[0],\
                                       "y":self.P[1],\
                                       "z":self.P[2],\
                                       "t":self.P[3],\
                                       "spd":self.M2_speed}) + "\n").encode())
    def send_pose(self, pose):
        """ 发送角度控制命令到机械臂 """
        cmd = {
            "T": 122, \
            "b": pose[0],\
            "s": pose[1],\
            "e": pose[2],\
            "h": pose[3],\
            "spd": self.M2_speed,\
            "acc": self.M2_acc
        }
        self.serial.write((json.dumps(cmd) + "\n").encode())

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("arm_teleop")

    arm = ArmTeleop("/dev/ttyUSB3", 115200)

    print("控制说明:")
    print("a/d : 第一关节 左/右")
    print("w/s : 第二关节 前/后")
    print("e/c : 第三关节 上/下")
    print("q/z : 夹爪 开/合")
    print("空格 : 停止（保持当前姿态）")
    print("CTRL-C 退出")

    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key == 'a':
                pose[0] += step_angle
            elif key == 'd':
                pose[0] -= step_angle
            elif key == 'w':
                pose[1] += step_angle
            elif key == 's':
                pose[1] -= step_angle
            elif key == 'e':
                pose[2] += step_angle
            elif key == 'c':
                pose[2] -= step_angle
            elif key == 'q':
                pose[3] += step_angle  # 夹爪张开
            elif key == 'z':
                pose[3] -= step_angle  # 夹爪闭合
            elif key == ' ':
                rospy.loginfo("保持当前位置")
            elif key == '\x03':  # CTRL-C
                break
            elif key == 'u':
                pose[4] += step_angle
            elif key == 'i':
                pose[4] -= step_angle
            #修改步长
            elif key == 'h':
                step_angle+=0.5
                print("步长为:"+step_angle)
            elif key == 'b':
                step_angle-=0.5
                print("步长为:"+step_angle)
            elif key == 't':
                print("请输入x y z :")
                x = float(input("x: "))
                y = float(input("y: "))
                z = float(input("z: "))
                arm.send_xyz(x, y, z)
                time.sleep(10)
            elif key == 'g':
                print("请输入pose :")
                pose[0] = float(input("pose_1: "))
                pose[1] = float(input("pose_2: "))
                pose[2] = float(input("pose_3: "))
                pose[3] = float(input("pose_4: "))
            elif key == 'p':
                times=30
                time=1
                print("执行矫正")
                dx,dy,Z= pixel_to_world(arm.vision_data[1],arm.vision_data[2],50)
                while abs(dx) > 5 or abs(dy) > 5 and time < times:
                    dx, dy, Z = pixel_to_world(arm.vision_data[1], arm.vision_data[2], 50)
                    to_joint(dx, dy)
                    time += 1
                    rospy.sleep(2)
                continue
            # 限制范围（避免超过关节极限）
            pose = [max(-180, min(180, p)) for p in pose]
            pose[3] = max(-45, min(45, pose[3]))
            # if key == 'v':
            #     # 发送姿态
            rospy.loginfo("发送关节角度: {}".format(pose))
            arm.send_pose(pose)

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
