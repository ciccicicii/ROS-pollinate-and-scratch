#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import time

import sys
from std_msgs.msg import Int32MultiArray
#from sympy import Q
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import Int32
from std_msgs.msg import Float32
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
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# 定义ANSI颜色码
COLOR_RED = '\033[91m'
COLOR_GREEN = '\033[92m'
COLOR_YELLOW = '\033[93m'
COLOR_BLUE = '\033[94m'
COLOR_PURPLE = '\033[95m'
COLOR_CYAN = '\033[96m'
COLOR_WHITE = '\033[97m'

# 定义ANSI样式码
STYLE_BOLD = '\033[1m'
STYLE_UNDERLINE = '\033[4m'
STYLE_RESET = '\033[0m'

PI = 3.14159

cmd_vel_topic = rospy.get_param('cmd_vel_topic','/cmd_vel')
cmd_vel_pub = rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)
imu_stop_pub = rospy.Publisher('stop_flag',Int8,queue_size=10)
stop_flag = 0
direction_pub = rospy.Publisher('direction', Int8, queue_size=10)
derection = 0  # 初始方向为直行
#音频驱动
def play_wav_file(file_path):
    """
    调用系统播放器播放WAV文件，指定音频设备
    :param file_path: WAV文件路径
    """
    # 检查文件是否存在
    if not os.path.exists(file_path):
        print("错误：文件不存在")
        sys.exit(1)

    # 检查文件是否是WAV格式
    if not file_path.lower().endswith('.wav'):
        print("错误：文件不是WAV格式")
        sys.exit(1)

    try:
        # 使用指定音频设备播放（添加 -D plughw:0,3 参数）
        subprocess.call(['aplay', '-D', 'plughw:0,3', file_path])
    except OSError as e:
        if e.errno == os.errno.ENOENT:
            print("错误：未找到 'aplay' 命令。请安装 alsa-utils：sudo apt-get install alsa-utils")
            sys.exit(1)
        else:
            raise
#播报
def aplay(path):
    play_wav_file(path)


class MOVE_ARRIVE:
    def time_delay(self,s):
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time<s):
            xyz=0
    #等待首次视觉有效数据
    def wait_for_vision_data(self):
        # 等待首次有效数据
        rospy.loginfo("Waiting for first valid vision data...")
        while self.vision_data is None:
            rospy.sleep(0.1)  # 等待数据
        print("First valid vision data received:")
    def vision_callback(self, msg):
        # 回调函数，接受视觉数据并存储
        self.vision_data = msg.data[0]  # 数据格式为 [flower_id, x_center, y_center, width, height]\
        self.mid=msg.data[1]
    def Qr_callback(self,msg):
        self.qr_array=msg.data
    def adjust_with_vision(self): #视觉位姿矫正
        retry = 0
        max_retry = 10  # 最多尝试时间约为5s
        while self.vision_data is not None or abs(self.mid - 320) > 5:  # 视觉中心偏移量
            if self.vision_data is not None:
                # offset = self.vision_data[1] - 320  # 计算与图像中心的偏移
                # speed = 0.05 if offset > 0 else -0.05  # 根据偏移决定转动方向
                # self.cmd_vel_msg.linear.x = 0  # 保持直线运动速度为0
                # self.cmd_vel_msg.angular.z = speed  # 调整旋转速度
                # self.cmd_vel_pub.publish(self.cmd_vel_msg)  # 发布控制命令
                # print("Adjusting")
                # rospy.sleep(0.1)  # 等待
                # 计算花朵偏移量，相对图像中心320像素
                offset = float(self.mid) - float(320)
                # 根据偏移量调整速度
                speed = abs(offset)*0.002
                speed = max(speed, 0.1)  # 限制最大速度为0.2 m/s
                # 如果花朵偏左，向右前进，否则向左前进
                self.cmd_vel_msg.linear.x = speed if offset > 0 else -speed
                # 发布调整命令
                print("Adjusting: offset = " + str(offset) + " speed = " + str(self.cmd_vel_msg.linear.x))
                cmd_vel_pub.publish(self.cmd_vel_msg)  # 发布控制命令
                rospy.sleep(0.1)  # 等待一段时间再调整
                self.cmd_vel_msg.linear.x = 0
                self.cmd_vel_msg.angular.z = 0
                cmd_vel_pub.publish(self.cmd_vel_msg)
                rospy.sleep(1)
            else: print("no vision data received yet")
            retry += 1
            if retry > max_retry:  # 如果尝试超过最大次数则退出
                rospy.logwarn("Max retries reached, aborting vision adjustment.")
                break
    def quaternion_to_euler(qw, qx, qy, qz):
        # 计算欧拉角的yaw
        t3 = +2.0 * (qw * qz + qx * qy)
        t4 = +1.0 - 2.0 * (qy**2 + qz**2)
        yaw_z = math.atan2(t3, t4)
        return  yaw_z
    # 机械臂位姿控制
    def arm_pose(self,pose):
        self.cmd_arm_msg.data = pose+2
        self.cmd_arm_pub.publish(self.cmd_arm_msg)
    def arm_detect_and_scratch(self,lr_flag,area_flag):
        # 机械臂采摘
        # 视觉检测
        # 检测左边的蔬菜
        # 机械臂采摘
        # 视觉检测
        # 是否采摘
        if area_flag: #A区
            l_see=2
            r_see=4
            l_scratch=3
            r_scratch=5
            l_yichu=6
            r_yichu=7
            l_reset=0
            r_reset=1
        else: #C区
            l_see=10
            r_see=12
            l_scratch=11
            r_scratch=13
            l_yichu=6
            r_yichu=9
            l_reset=0
            r_reset=1
        if(lr_flag == 0): #左边蔬菜
            self.arm_pose(l_see)
            print("机械臂移动到左边视觉识别处")
            self.time_delay(7)

            #视觉矫正
            #self.adjust_with_vision()
            if(self.vision_data in self.vefru_arr):
                rospy.loginfo("未成熟蔬菜，不采摘")
                aplay(self.AC_ve_path[13])
                self.arm_pose(l_reset) #机械臂归位
                self.time_delay(3)
            elif (self.vision_data == 12):
                aplay(self.AC_ve_path[self.vision_data])
                rospy.loginfo("坏蔬菜,移除")
                self.arm_pose(l_yichu) #抓取抬起放置
                self.time_delay(7)
            else:
                rospy.loginfo("成熟蔬菜，采摘")
                aplay(self.AC_ve_path[self.vision_data])
                self.ve_class.append(self.vision_data)
                self.arm_pose(l_scratch) #抓取抬起放置
                self.time_delay(8)
        else: #右边蔬菜
            self.arm_pose(r_see)
            print("机械臂移动到右边视觉识别处")
            self.time_delay(7)
            if(self.vision_data in self.vefru_arr):
                rospy.loginfo("未成熟蔬菜，不采摘")
                aplay(self.AC_ve_path[13])
                self.arm_pose(r_reset) #机械臂归位
                self.time_delay(3)
            elif (self.vision_data == 12):
                rospy.loginfo("坏蔬菜,移除")
                aplay(self.AC_ve_path[self.vision_data])
                self.arm_pose(r_yichu)
                self.time_delay(7)
            else:
                rospy.loginfo("成熟蔬菜，采摘")
                aplay(self.AC_ve_path[self.vision_data])
                self.ve_class.append(self.vision_data)
                self.arm_pose(r_scratch) #抓取抬起放置
                self.time_delay(8)
    def task_A(self):
        print(COLOR_YELLOW+ ">>>>>Start Task A." + STYLE_RESET)
        #self.set_yaw_pub.publish(self.angles) #重置初始角度
        for i in range(5):
            print(COLOR_YELLOW+ ">>>>>>>>>>>Task A No"+str(i+1) +" ..."+ STYLE_RESET)
            if i == 0:
                stop_flag = 1
            if i<4:
                stop_flag = 1
            elif i == 4:
                stop_flag = -1
            imu_stop_pub.publish(Int8(stop_flag))
            cmd_vel_pub.publish(self.cmd_vel_msg)
            print(COLOR_YELLOW+ ">>>>>>>>>>>Move. Speed is "+str(self.linear_x) +"m/s. Move time is "\
                                        +str(self.goA_along_time[self.task_A_go_along[i]][0])+"s. Stop time is "\
                                            +str(self.goA_along_time[self.task_A_go_along[i]][1])+"s."+ STYLE_RESET)
            self.time_delay(self.goA_along_time[self.task_A_go_along[i]][0])
            stop_flag = 0
            imu_stop_pub.publish(Int8(stop_flag))
            print(COLOR_YELLOW+ ">>>>>>>>>>>Stop."+ STYLE_RESET)
            if(i<4):
                print("Start scratch...")
                #机械臂采摘
                #视觉检测
                #检测左边并抓取的蔬菜
                print("检测并抓取左边的")
                self.arm_detect_and_scratch(0,1)
                #检测并抓取右边的蔬菜
                print("检测并抓取右边的")
                self.arm_detect_and_scratch(1,1)
            rospy.loginfo("scratch finished.")
            self.time_delay(1)
        print(COLOR_YELLOW+ ">>>>>Task A finished." + STYLE_RESET)

    def task_C(self):
        print(COLOR_YELLOW+ ">>>>>Start Task C." + STYLE_RESET)
        for i in range(6):
            print(COLOR_YELLOW+ ">>>>>>>>>>>Task C No"+str(i+1) +" ..."+ STYLE_RESET)
            if i == 0: #右转、
                self.time_delay(4)
                # self.cmd_vel_msg.linear.x = 0
                # self.cmd_vel_msg.angular.z = self.anglular_z
                # cmd_vel_pub.publish(self.cmd_vel_msg)
                # self.time_delay(self.goC_along_time[self.task_C_go_along[i]][0])
                # self.cmd_vel_msg.linear.x = 0
                # self.cmd_vel_msg.angular.z = 0
                # cmd_vel_pub.publish(self.cmd_vel_msg)
                derection = 1  # 设置方向为右直行
                direction_pub.publish(Int8(derection))
                self.time_delay(1)

                stop_flag = 2
                imu_stop_pub.publish(Int8(stop_flag))
                self.time_delay(4)
                stop_flag = 0
                imu_stop_pub.publish(Int8(stop_flag))
            elif i == 1: #直行进行扫码
                derection = 1  # 设置方向为右直行
                direction_pub.publish(Int8(derection))
                self.time_delay(1)

                stop_flag = 1
                imu_stop_pub.publish(Int8(stop_flag))
                self.time_delay(self.goC_along_time[self.task_C_go_along[i]][0])
                stop_flag = 0
                imu_stop_pub.publish(Int8(stop_flag))
                #扫码

                # self.arm_pose(2)
                # self.time_delay(7)
                # stop_flag = 3
                # imu_stop_pub.publish(Int8(stop_flag))
                # self.adjust_with_vision()  # 视觉矫正
                # self.arm_pose(0)


                self.time_delay(3)



                # stop_flag = 0
                # imu_stop_pub.publish(Int8(stop_flag))

            elif i == 2: #直行
                derection = 1  # 设置方向为右直行
                direction_pub.publish(Int8(derection))
                self.time_delay(1)

                stop_flag = 1
                imu_stop_pub.publish(Int8(stop_flag))
                self.time_delay(self.goC_along_time[self.task_C_go_along[i]][0])
                stop_flag = 0
                imu_stop_pub.publish(Int8(stop_flag))
            elif i == 3: #左转
                self.time_delay(2)
                # self.cmd_vel_msg.linear.x = 0
                # self.cmd_vel_msg.angular.z = -self.anglular_z
                # cmd_vel_pub.publish(self.cmd_vel_msg)
                # self.time_delay(self.goC_along_time[self.task_C_go_along[i]][0])
                # self.cmd_vel_msg.linear.x = 0
                # self.cmd_vel_msg.angular.z = 0
                # cmd_vel_pub.publish(self.cmd_vel_msg)
                derection = 0  # 设置方向为右直行
                direction_pub.publish(Int8(derection))
                self.time_delay(4)

                stop_flag = 2
                imu_stop_pub.publish(Int8(stop_flag))
                self.time_delay(4)
                stop_flag = 0
                imu_stop_pub.publish(Int8(stop_flag))
            elif i == 4: #直行到收集点进行播报
                derection = 0  # 设置方向为直行
                direction_pub.publish(Int8(derection))
                self.time_delay(1)

                stop_flag = 1
                imu_stop_pub.publish(Int8(stop_flag))
                self.time_delay(self.goC_along_time[self.task_C_go_along[i]][0])
                stop_flag = 0
                imu_stop_pub.publish(Int8(stop_flag))
                #播报A区收获情况
                play_wav_file("/home/epaicar/sounds/a.wav")
                self.time_delay(2)
                for j in self.ve_class:
                    if j not in self.ve_path:
                        continue
                    play_wav_file(self.ve_path[j])
                    self.time_delay(1)
            elif i == 5: #返回到12点
                stop_flag = -1
                imu_stop_pub.publish(Int8(stop_flag))
                self.time_delay(self.goC_along_time[self.task_C_go_along[i]][0])
                stop_flag = 0
                imu_stop_pub.publish(Int8(stop_flag))

            print(COLOR_YELLOW+ ">>>>>>>>>>>Stop."+ STYLE_RESET)
            self.time_delay(2)
        print(COLOR_YELLOW+ ">>>>>Task C Start." + STYLE_RESET)
        #C区路径规划
        road, raw = 2,4
        for q in self.qr_array:
            flag = 0
            road_2, raw_2 = self.Plan.point_map[q]
            if q in self.Plan.shared: flag = 1
            self.plan_c_action(raw, road, q, raw_2, road_2, flag)
            raw = raw_2
            if flag!=1:
                road = road_2
            rospy.loginfo("到达目标点(%d,%d 序列%d)", road, raw, q)
            #机械臂采摘 左识别10 采摘11，右识别12 采摘13
            if q in self.is_left:
                self.arm_detect_and_scratch(0, 0)
            elif q in self.is_right:
                self.arm_detect_and_scratch(1, 0)
            elif q in self.is_share and road == 1:
                self.arm_detect_and_scratch(1,0)
            elif q in self.is_share and road == 2:
                self.arm_detect_and_scratch(0,0)

            rospy.loginfo("scratch finished.")
            rospy.sleep(1)  # 等待3秒

    def plan_c_action(self, raw, road,q,raw_2,road_2,flag):
        #同通道化
        if road == road_2 or flag == 1:
            # 同一通道
            dr = raw_2 - raw
            if dr > 0: self.Plan.backward(dr)
            elif dr < 0 : self.Plan.forward(-dr)
        else:
            # 转换通道
            # 退到末尾行
            self.Plan.backward(4 - raw)
            rospy.sleep(1)
            # 出通道
            self.Plan.backward_small()
            rospy.sleep(1)
            # 转换通道
            if road < road_2:
                self.Plan.change_road(1)
            else:
                self.Plan.change_road(2)
            # 进入通道
            self.Plan.forward_small()
            rospy.sleep(1)
            # 进入目标行
            self.Plan.forward(4-raw_2)
            rospy.sleep(1)

    def __init__(self):

        self.cmd_arm_topic = rospy.get_param('cmd_arm_topic','/arm_cmd')###teb
        #订阅imu角度
        #rospy.Subscriber("/imu_data",Imu,ImuCallBack)
        self.goA_along_time = rospy.get_param("goA_along_time",[[5.0,30.0],[5.0,30.0],[5.0,30.0]])
        self.goB_along_time = rospy.get_param("goB_along_time",[[5.0,30.0],[5.0,30.0],[5.0,30.0]])
        self.goC_along_time = rospy.get_param("goC_along_time",[[5.0,30.0],[5.0,30.0],[5.0,30.0]])
        self.task_A_go_along = rospy.get_param("task_A_go_along",[1,2,2,2,2,2,3,4,5])
        self.task_B_go_along = rospy.get_param("task_B_go_along",[1,2,2,2,2,2,3,6])
        self.task_C_go_along = rospy.get_param("task_C_go_along",[1,2,2,2,2,2,3,4,5])

        self.cmd_arm_pub = rospy.Publisher(self.cmd_arm_topic,Int32,queue_size=10)
        self.set_yaw_pub = rospy.Publisher("set_yaw",Float32,queue_size=10)
        self.angles = 0.0
        #未成熟蔬菜数组编号数组
        self.vefru_arr=[1,3,5,7,9,11]
        #C区采摘判断
        self.is_left = [1,2,3,4]
        self.is_share = [5,6,7,8]
        self.is_right = [9,10,11,12]
        #A区采摘蔬菜种类
        self.ve_class=[]
        #蔬菜编号字典号
        self.ve_path={4:"/home/epaicar/sounds/tomato.wav",5:"/home/epaicar/sounds/Otomato.wav"
                      , 6:"/home/epaicar/sounds/onion.wav",7:"/home/epaicar/sounds/Oonion.wav"
                      ,8:"/home/epaicar/sounds/pepper.wav",9:"/home/epaicar/sounds/Opepper.wav"
                      ,10:"/home/epaicar/sounds/pumpkin.wav",11:"/home/epaicar/sounds/Opumpkin.wav"
                      ,12:"/home/epaicar/sounds/bad.wav",13:"/home/epaicar/sounds/B.wav"}
        self.AC_ve_path={4:"/home/epaicar/sounds/Ftomato.wav",5:"/home/epaicar/sounds/Otomato.wav"
                         , 6:"/home/epaicar/sounds/Fonion.wav",7:"/home/epaicar/sounds/Oonion.wav"
                         ,8:"/home/epaicar/sounds/Fpepper.wav",9:"/home/epaicar/sounds/Opepper.wav"
                         ,10:"/home/epaicar/sounds/Fpumpkin.wav",11:"/home/epaicar/sounds/Opumpkin.wav"
                         ,12:"/home/epaicar/sounds/bad.wav",13:"/home/epaicar/sounds/B.wav"}
        # 订阅雷达数据，获取左右两侧障碍物的最小距离
        self.left_dis_angle = 0
        self.right_dis_angle = 180
        self.min_dis = [10,10]
        #rospy.Subscriber('/scan', LaserScan, self.get_laser_min_dis)

        self.cmd_vel_msg = Twist()
        self.linear_x = rospy.get_param('linear_x',0.2)
        self.anglular_z = rospy.get_param('anglular_z',2.5)
        self.cmd_arm_msg = Int32()

        #self.timer_vel = rospy.Timer(rospy.Duration(0.05),self.ljy_vel)###定时器发布所有速度
        # 订阅视觉数据
        rospy.Subscriber('/yolov5_sign', Int32MultiArray, self.vision_callback)
        self.vision_data = -1
        #Qr二维码订阅
        rospy.Subscriber('/Qr_scan', Int32MultiArray, self.Qr_callback)
        self.qr_array = [1,5,11,9,10,4,3,12]  # 默认值，实际使用中会被回调函数更新
        self.stop_flag = 0
        self.mid=320
        self.Plan = PlanC()
        # 确保收到有效的视觉
        self.wait_for_vision_data()
    def RobotStart(self):
        #while not rospy.is_shutdown():
        #语音播报
        aplay('/home/epaicar/sounds/TeamName.wav')
        #self.time_delay(2.0)
        self.task_A()
        #self.task_B()
        self.task_C()

#C区路径规划
class PlanC:
    def __init__(self):
        #点数映射
        self.point_map = {
            1:(1,1), 2:(1,2),
            3:(1,3), 4:(1,4),
            5:(1,1), 6:(1,2),
            7:(1,3), 8:(1,4),
            9:(2,1), 10:(2,2),
            11:(2,3), 12:(2,4),
            }
        self.shared = [5,6,7,8]
        self.actions = []
        #速度参数
        self.forward_time = rospy.get_param("forward_time", 1)
        self.backward_time = rospy.get_param("backward_time", 1)
        self.forward_small_time = rospy.get_param("forward_small_time", 0.5)
        self.backward_small_time = rospy.get_param("backward_small_time", 0.5)
        self.forward_big_time = rospy.get_param("forward_big_time", 2)
        self.turn_left_time = rospy.get_param("turn_left_time", 1.5)
        self.turn_right_time = rospy.get_param("turn_right_time", 1.5)
        self.cmd_vel_msg = Twist()
        self.linear_x = rospy.get_param('linear_x',0)
        self.anglular_z = rospy.get_param('anglular_z',1.5)
    def time_delay(self,s):
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time<s):
            xyz=0
    def forward(self,n):
        self.actions += ["forward_1"]*n
        rospy.loginfo("前进%d", n)
        if n == 0:
            return
        stop_flag = 1
        imu_stop_pub.publish(Int8(stop_flag))
        rospy.sleep(self.forward_time*n)
        stop_flag = 0
        imu_stop_pub.publish(Int8(stop_flag))
    def backward(self,n):
        self.actions += ["backward_1"]*n
        rospy.loginfo("后退%d", n)
        if n == 0:
            return
        stop_flag = -1
        imu_stop_pub.publish(Int8(stop_flag))
        rospy.sleep(self.backward_time*n)
        stop_flag = 0
        imu_stop_pub.publish(Int8(stop_flag))
    def forward_small(self):
        self.actions.append("forward_small")
        rospy.loginfo("进入通道")
        stop_flag = 1
        imu_stop_pub.publish(Int8(stop_flag))
        rospy.sleep(self.forward_small_time)
        stop_flag = 0
        imu_stop_pub.publish(Int8(stop_flag))
    def backward_small(self):
        self.actions.append("backward_small")
        rospy.loginfo("退出通道")
        stop_flag = -1
        imu_stop_pub.publish(Int8(stop_flag))
        rospy.sleep(self.backward_small_time)
        stop_flag = 0
        imu_stop_pub.publish(Int8(stop_flag))
    def change_road(self, road):
        if road == 1:
            self.time_delay(0.5)
            self.actions.append("change_road_1")
            derection = 1  # 设置方向为右直行
            direction_pub.publish(Int8(derection))
            self.time_delay(4)
            #转向
            stop_flag = 2
            imu_stop_pub.publish(Int8(stop_flag))
            self.time_delay(4)
            stop_flag = 0
            imu_stop_pub.publish(Int8(stop_flag))
            self.time_delay(1)
            #直行
            stop_flag = 1
            imu_stop_pub.publish(Int8(stop_flag))
            rospy.sleep(self.forward_big_time)
            stop_flag = 0
            imu_stop_pub.publish(Int8(stop_flag))
            self.time_delay(5)
            print("转向")
            #转向
            derection = 0  # 设置方向为右直行
            direction_pub.publish(Int8(derection))
            self.time_delay(2)
            stop_flag = 2
            imu_stop_pub.publish(Int8(stop_flag))
            self.time_delay(4)
            stop_flag = 0
            imu_stop_pub.publish(Int8(stop_flag))

        else:
            self.time_delay(0.5)
            self.actions.append("change_road_2")
            derection = -1  # 设置方向为左直行
            direction_pub.publish(Int8(derection))
            self.time_delay(4)
            #转向
            stop_flag = 2
            imu_stop_pub.publish(Int8(stop_flag))
            self.time_delay(4)
            stop_flag = 0
            imu_stop_pub.publish(Int8(stop_flag))
            self.time_delay(1)
            #直行
            stop_flag = 1
            imu_stop_pub.publish(Int8(stop_flag))
            rospy.sleep(self.forward_big_time)
            stop_flag = 0
            imu_stop_pub.publish(Int8(stop_flag))
            self.time_delay(5)
            print("转向")
            #转向
            derection = 0  # 设置方向为直行
            direction_pub.publish(Int8(derection))
            self.time_delay(2)
            stop_flag = 2
            imu_stop_pub.publish(Int8(stop_flag))
            self.time_delay(4)
            stop_flag = 0
            imu_stop_pub.publish(Int8(stop_flag))

    def action(self, seq,start):
        road, raw = self.point_map[start]
        for q in seq:
            flag = 0
            road_2, raw_2 = self.point_map[q]
            #同通道化
            if q in self.shared: flag = 1
            if road == road_2 or flag == 1:
                # 同一通道
                dr = raw_2 - raw
                if dr > 0: self.backward(dr)
                elif dr < 0 : self.forward(-dr)
            else:
                # 转换通道
                # 退到末尾行
                self.backward(4 - raw)
                # 出通道
                self.backward_small()
                # 转换通道
                if road < road_2:
                    self.change_road(1)
                else:
                    self.change_road(2)
                # 进入通道
                self.forward_small()
                # 进入目标行
                self.forward(4-raw_2)
            raw = raw_2
            #添加判断避免通道 同化
            if flag != 1:
                road = road_2
            rospy.loginfo("到达目标点(%d,%d 序列%d)", road, raw,q)
#main function
if __name__=="__main__":
    try:
        rospy.init_node('compitation_node',anonymous=True)
        rospy.loginfo('compitation_irrigate_node instruction start...')
        MA = MOVE_ARRIVE()
        MA.RobotStart()
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")
