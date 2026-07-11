#!/usr/bin/python
#coding=utf-8

import roslib
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray , MultiArrayLayout, MultiArrayDimension
from geometry_msgs.msg import Twist
import threading
import json
import serial
import os
import sys
import select
import termios
import tty
import re

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

# 保存终端属性
settings = termios.tcgetattr(sys.stdin)
record_mode = True
set_pose_mode = False                                              # True: 以关节角度模式录制轨迹，False: 以三维坐标点模式XYZT录制轨迹，默认为True，可以通过键盘q键进行切换
write_pose_flag = 0                                                   #按下w键等于1，开始录制轨迹，轨迹数组保存在pose_array.yaml里面
write_count = 0                                                       #以角度方式录制轨迹的索引号
XYZT_count = 0                                                        #以三维坐标XYZT方式录制轨迹的索引号
class arm_driver():

    def __init__(self):

        rospy.init_node('RoArm_M2', anonymous=True)

        self.P = Float32MultiArray()
        self.degree_control = 122                                                     #102对应弧度控制，122对应°控制，默认°控制。
        self.seted_pose =  [0.00, 0.00, -80.00, 180.00]
        self.XYZT_control = 104                                                       #104对应三维坐标XYZT控制

        self.M2_init_pose = [0.00, -20.00, -15.00, 0.00]                               #初始化位置数组:依次对应基关节，肩关节，肘关节，末端关节

        self.arm_current_pose = [0.00, -40.00, -10.00, 0.00]                           #第二线程不断获取机械臂各个关节当前位置

        self.M2_current_XYZt = [312.232, 6.227, 231.807, 3.14]                        #机械臂末端当前的XYZ位置以及末端夹爪t的角度radius

        self.M2_speed = rospy.get_param("M2_speed",40)                                #配置关节速度参数，默认30°/S

        self.M2_acc = rospy.get_param("M2_acc",20)                                    #配置关节加速度参数，默认20°/S

        self.arm_serial_device = rospy.get_param("M2_serial","/dev/ttyUSB2")          #配置串口号
        #self.arm_serial_device = "/dev/ttyUSB2"

        self.M2_sub_coord_topic = rospy.get_param("M2_sub_coord_topic","/arm_coord")  #配置三维空间坐标点数据话题

        self.M2_sub_cmd_topic = rospy.get_param("M2_sub_cmd_topic","/arm_cmd")        #配置命令/arm_cmd话题

        self.pub_pose_topic = rospy.get_param("M2_pub_pose","/arm_pose")              #配置命令/arm_pose当前机械臂各个关节角度话题

        self.M2_pose_XYZT = rospy.get_param("M2_pose_XYZT","/M2_pose_XYZT")           #配置命令/M2_XYZT当前机械臂xyzt坐标参数话题

        self.M2_target_pose = [[[]]]                                                  #定义目标位置各个关节角度数组变量

        self.M2_target_pose = rospy.get_param("M2_target_pose",[[[]]])                       #加载目标位置各个关节角度数组

        self.M2_torque = [84.0, 36.0, 24.0, 32.0]                                     #存放各个关节力矩大小

        self.M2_torque_topic  = rospy.get_param("M2_torque_topic","/M2_torque")       #加载力矩大小话题

        self.M2_target_num = []                                                       #定义目标位置关节角度数组长度大小

        self.M2_target_num = rospy.get_param("M2_target_num",[])                         #加载目标位置各个关节角度数组大小

        self.arm_delay_time = rospy.get_param("arm_delay_time",0.1)                   #定义延时时间

        self.arm_pose_array = []

        self.read_array_from_yaml = []    #定义二位数组读取yaml文件参数

        self.serial = serial.Serial(self.arm_serial_device,115200,write_timeout=0.01)  #timeout=0.01,write_timeout=0.01  ,打开串口，波特率115200，超时0.02秒

        self.trajectory = rospy.get_param("M2_trajectory",10)
        self.sub_coord = rospy.Subscriber(self.M2_sub_coord_topic,Float32MultiArray,self.set_coord_callback,queue_size=20)  #收到三维坐标点话题回调函数

        self.sub_cmd = rospy.Subscriber(self.M2_sub_cmd_topic,Int32,self.listener_callback,queue_size=20)                   #收到/arm_cmd话题回调函数

        self.sub_write_cmd = rospy.Subscriber("/read_array",Int32,self.read_array_callback,queue_size=20)
        self.set_pose_cmd = rospy.Subscriber("/set_pose",Float32MultiArray,self.set_pose_callback,queue_size=20)
        self.pub_pose = rospy.Publisher(self.pub_pose_topic,Float32MultiArray,queue_size=10)                                #发布当前的各个关节角度话题数组，单位°

        self.pub_XYZT_pose = rospy.Publisher(self.M2_pose_XYZT,Float32MultiArray,queue_size=10)                             #发布当前的XYZT话题数组

        self.pub_Torque = rospy.Publisher(self.M2_torque_topic,Float32MultiArray,queue_size=10)                             #发布当前的力矩话题数组

        self.timer_pub_angle = rospy.Timer(rospy.Duration(100.0/1000),self.timerPubAngleCB)                                 #定时器即时发布当前机械臂位姿

        self.serial_init()

        self.serial_recv_thread = threading.Thread(target=self.read_serial)
        self.serial_recv_thread.daemon = True
        self.serial_recv_thread.start()
        self.time_delay(2.0)
    def serial_init(self):
        #机械臂初始化
        rospy.loginfo("串口打开成功，等待机械臂 MCU 就绪...")
        rospy.sleep(2.0)
        # 清空缓存
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
        rospy.loginfo("串口缓存已清空")
        # 发送一个测试包，触发通信
        try:
            self.serial.write(b'\n')
            self.serial.flush()
        except serial.SerialTimeoutException:
            rospy.logwarn("启动阶段超时，刷新通信状态")
            self.serial.reset_output_buffer()
            self.serial.reset_input_buffer()
        self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_init_pose[0],\
                                       "s":self.M2_init_pose[1],\
                                       "e":self.M2_init_pose[2],\
                                       "h":self.M2_init_pose[3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
        print(COLOR_GREEN + ">>>>>Current Pose degree:" + str(self.arm_current_pose) + STYLE_RESET)
    def read_serial(self):
        pose_msg = Float32MultiArray()
        XYZT_msg = Float32MultiArray()
        Torque_msg = Float32MultiArray()
        layout = MultiArrayLayout()
        dim = MultiArrayDimension()
        dim.label = "bseh"
        dim.size = 4
        layout.dim.append(dim)

        while not rospy.is_shutdown():

            length = self.serial.in_waiting
            if length:
                reading = self.serial.readline().decode('utf-8',errors='ignore')
                if len(reading)>5:
                    match=re.findall(r'"T":1051',reading)
                    # print(COLOR_PURPLE + "----->reading:" + str(reading) + "count:" + str(count_test) + STYLE_RESET)
                    if len(match)>0:
                        pattern=re.compile(r'(?<="b":)-?\d+(?:\.\d+)?')
                        if(len(pattern.findall(reading))>0):
                            self.arm_current_pose[0] = (float(pattern.findall(reading)[0]))/3.14159*180
                        pattern=re.compile(r'(?<="s":)-?\d+(?:\.\d+)?')
                        if(len(pattern.findall(reading))>0):
                            self.arm_current_pose[1] = -(float(pattern.findall(reading)[0]))/3.14159*180
                        pattern=re.compile(r'(?<="e":)-?\d+(?:\.\d+)?')
                        if(len(pattern.findall(reading))>0):
                            self.arm_current_pose[2] = (float(pattern.findall(reading)[0]))/3.14159*180-90
                        pattern=re.compile(r'(?<="t":)-?\d+(?:\.\d+)?')
                        if(len(pattern.findall(reading))>0):
                            self.M2_current_XYZt[3] = float(pattern.findall(reading)[0])
                            self.arm_current_pose[3] = (float(pattern.findall(reading)[0]))/3.14159*180
                        pattern=re.compile(r'(?<="x":)-?\d+(?:\.\d+)?')
                        if(len(pattern.findall(reading))>0):
                            self.M2_current_XYZt[0] = float(pattern.findall(reading)[0])
                        pattern=re.compile(r'(?<="y":)-?\d+(?:\.\d+)?')
                        if(len(pattern.findall(reading))>0):
                            self.M2_current_XYZt[1] = float(pattern.findall(reading)[0])
                        pattern=re.compile(r'(?<="z":)-?\d+(?:\.\d+)?')
                        if(len(pattern.findall(reading))>0):
                            self.M2_current_XYZt[2] = float(pattern.findall(reading)[0])
                        pattern=re.compile(r'(?<="torB":)-?\d+(?:\.\d+)?')
                        if(len(pattern.findall(reading))>0):
                            self.M2_torque[0] = float(pattern.findall(reading)[0])
                        pattern=re.compile(r'(?<="torS":)-?\d+(?:\.\d+)?')
                        if(len(pattern.findall(reading))>0):
                            self.M2_torque[1] = float(pattern.findall(reading)[0])
                        pattern=re.compile(r'(?<="torE":)-?\d+(?:\.\d+)?')
                        if(len(pattern.findall(reading))>0):
                            self.M2_torque[2] = float(pattern.findall(reading)[0])
                        pattern=re.compile(r'(?<="torH":)-?\d+(?:\.\d+)?')
                        if(len(pattern.findall(reading))>0):
                            self.M2_torque[3] = float(pattern.findall(reading)[0])
                        #print(COLOR_GREEN + ">>>>>Current Pose radius:" + str(self.arm_current_pose) + STYLE_RESET)


                pose_msg.layout = layout
                pose_msg.data = self.arm_current_pose
                self.pub_pose.publish(pose_msg)

                XYZT_msg.data = self.M2_current_XYZt
                self.pub_XYZT_pose.publish(XYZT_msg)

                Torque_msg.data = self.M2_torque
                self.pub_Torque.publish(Torque_msg)



    def timerPubAngleCB(self,event):
        global write_pose_flag
        global write_count
        self.serial.write((json.dumps({"T":105})+'\n').encode())
        if record_mode:
            if write_pose_flag == 1:
                self.arm_pose_array.append(self.arm_current_pose[:])
                print(COLOR_GREEN + "Now You are starting to record the trajectory with degree------" + str(self.arm_current_pose) + STYLE_RESET)
            elif  write_pose_flag == 2:
                #print(COLOR_YELLOW + ">>>>> self.arm_pose_array:" + str(self.arm_pose_array) + STYLE_RESET)
                print(COLOR_YELLOW + "OK! You have wrote the {} trajectory with degree successfully !".format(write_count) + STYLE_RESET)
                os.system("echo write_array{}: {} >> /home/epaicar/talos_ws/src/agrc_base_arm/config/pose_array.yaml".format(write_count,self.arm_pose_array))
                rospy.set_param("write_array{}".format(write_count),self.arm_pose_array)
                self.arm_pose_array = []
                write_pose_flag = 0
        else:
            if write_pose_flag == 1:
                self.arm_pose_array.append(self.M2_current_XYZt[:])
                print(COLOR_GREEN + "Now You are starting to record the trajectory with XYZT------" + str(self.M2_current_XYZt) + STYLE_RESET)
            elif  write_pose_flag == 2:
                #print(COLOR_YELLOW + ">>>>> self.arm_pose_array:" + str(self.arm_pose_array) + STYLE_RESET)
                print(COLOR_YELLOW + "OK! You have wrote the {} trajectory with XYZT successfully !".format(write_count) + STYLE_RESET)
                os.system("echo XYZT_array{}: {} >> /home/epaicar/talos_ws/src/agrc_base_arm/config/pose_array_XYZT.yaml".format(XYZT_count,self.arm_pose_array))
                rospy.set_param("XYZT_array{}".format(XYZT_count),self.arm_pose_array)
                self.arm_pose_array = []
                write_pose_flag = 0
        #第二关节限幅在-75~+75°之间，第三关节限幅在15°~165°之间
        # if (abs(self.arm_current_pose[1])>=80)  or (abs(self.arm_current_pose[2])<=10) or (abs(self.arm_current_pose[2])>=170):
        #     print(COLOR_YELLOW + "emergency!!!!!!!!!!!!!!!!!!!!!!" + STYLE_RESET)
        #     print(COLOR_YELLOW + "start reset the initial position------" + STYLE_RESET)
        #     self.serial.write((json.dumps({"T":self.degree_control,\
        #                                "b":self.M2_init_pose[0],\
        #                                "s":self.M2_init_pose[1],\
        #                                "e":self.M2_init_pose[2],\
        #                                "h":self.M2_init_pose[3],\
        #                                "spd":self.M2_speed,\
        #                                "acc":self.M2_acc}) + "\n").encode())
        #     self.time_delay(0.3)

    def time_delay(self,s):
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time<s):
            xyz=0

    def set_coord_callback(self,msg):
        self.P = msg.data                       #104代表三位坐标目标点控制方式，x y z是坐标，t是末端角度，spd是速度
        self.serial.write((json.dumps({"T":104, \
                                       "x":self.P[0],\
                                       "y":self.P[1],\
                                       "z":self.P[2],\
                                       "t":self.P[3],\
                                       "spd":self.M2_speed}) + "\n").encode())
    def set_pose_callback(self,msg):
        print(COLOR_GREEN + ">>>>>set_pose_callback:" + str(msg.data) + STYLE_RESET)
        self.seted_pose = msg.data
    def listener_callback(self, msg):
        a = msg.data

        if a == 1:
            # init pos
            self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_init_pose[0],\
                                       "s":self.M2_init_pose[1],\
                                       "e":self.M2_init_pose[2],\
                                       "h":self.M2_init_pose[3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
        elif a == 2:
            # right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 3:
            # left work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)

        elif a == 4:
            # left & right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 5:
            # left & right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 6:
            # left & right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 7:
            # left & right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 8:
            # left & right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 9:
            # left & right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 10:
            # left & right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 11:
            # left & right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 12:
            # left & right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 13:
            # left & right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 14:
            # left & right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 15:
            # left & right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 16:
            # left & right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 17:
            # left & right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 18:
            # left & right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 19:
            # left & right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 20:
            # left & right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 21:
            # left & right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 22:
            # left & right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 23:
            # left & right work
            for i in range(self.M2_target_num[a-2]):
                self.serial.write((json.dumps({"T":self.degree_control,\
                                       "b":self.M2_target_pose[a-2][i][0],\
                                       "s":self.M2_target_pose[a-2][i][1],\
                                       "e":self.M2_target_pose[a-2][i][2],\
                                       "h":self.M2_target_pose[a-2][i][3],\
                                       "spd":self.M2_speed,\
                                       "acc":self.M2_acc}) + "\n").encode())
                self.time_delay( self.arm_delay_time)
        elif a == 24:
            print(COLOR_GREEN + ">>>>>Current Pose degree:" + str(self.arm_current_pose) + STYLE_RESET)           #打印实时机械臂角度数组

        elif a == 99:
            # ALL_TORQUE_OFF: {"T":8,"P1":0}
            self.serial.write((json.dumps({"T":210,"cmd":0})+'\n').encode())                                      #关闭扭矩锁
        elif a == 100:
            # ALL_TORQUE_ON: {"T":8,"P1":1}
            self.serial.write((json.dumps({"T":210,"cmd":1})+'\n').encode())                                      #开启扭矩锁
        elif a == 101:
            self.serial.write((json.dumps({"T":112,"mode":1,"b":60,"s":100,"e":35,"h":35})+'\n').encode())        #开启机械臂弹力自适应
        elif a == 102:
            self.serial.write((json.dumps({"T":112,"mode":0,"b":1000,"s":1000,"e":1000,"h":1000})+'\n').encode()) #关闭机械臂弹力自适应
    def read_array_callback(self, msg):
        b = msg.data
        if record_mode:
            temp_array = rospy.get_param("write_array{}".format(b),[[0.00, 0.00, 90.00, 180.00],\
                                                                    [0.00, 0.00, 90.00, 180.00]])
            #temp_array_size = len(temp_array)
            for array in temp_array:
                self.serial.write((json.dumps({"T":self.degree_control,\
                                            "b":array[0],\
                                            "s":array[1],\
                                            "e":array[2],\
                                            "h":array[3],\
                                            "spd":self.M2_speed,\
                                            "acc":self.M2_acc}) + "\n").encode())
                print(COLOR_YELLOW + ">>>>>array :" + str(array) + STYLE_RESET)

                self.time_delay(0.2)
        else:
            temp_array = rospy.get_param("XYZT_array{}".format(b),[[312.232, 6.227, 231.807, 3.14],\
                                                                   [312.232, 6.227, 231.807, 3.14]])
            #temp_array_size = len(temp_array)
            for array in temp_array:
                self.serial.write((json.dumps({"T":self.XYZT_control, \
                                       "x":array[0],\
                                       "y":array[1],\
                                       "z":array[2],\
                                       "t":array[3],\
                                       "spd":self.M2_speed}) + "\n").encode())
                print(COLOR_YELLOW + ">>>>>array :" + str(array) + STYLE_RESET)

                self.time_delay(0.2)

    def reset_pose(self,pose):
        #串口发布机械臂复位命令
        self.serial.write((json.dumps({
            "T": self.degree_control,
            "b" : pose[0],
            "s" : pose[1],
            "e" : pose[2],
            "h" : pose[3],
            "spd" : self.M2_speed,
            "acc" : self.M2_acc
        })+ "\n").encode())
        print("机械臂已移动到自定义位置:" + str(pose))
if __name__ == '__main__':
    try:
        my_arm = arm_driver()
        # 设置循环频率
        rate = rospy.Rate(10)  # 10Hz
        LED = False
        while not rospy.is_shutdown():
            # 设置终端属性，以便实时读取键值
            tty.setcbreak(sys.stdin.fileno())
            # tty.setraw(sys.stdin.fileno())
            if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
                # 从终端读取键值
                key = sys.stdin.read(1)
                if key == 'r':
                    my_arm.serial.write((json.dumps({"T":210,"cmd":0})+'\n').encode())                                        #关闭扭矩锁
                    print(COLOR_GREEN + "关闭扭矩锁" + STYLE_RESET)
                elif key == 't':
                    my_arm.serial.write((json.dumps({"T":210,"cmd":1})+'\n').encode())                                        #开启扭矩锁
                    print(COLOR_GREEN + "开启扭矩锁" + STYLE_RESET)
                elif key == 'y':
                    print(COLOR_GREEN + ">>>>>Current Pose degree:" + str(my_arm.arm_current_pose) + STYLE_RESET)             #查询实时的机械臂角度
                elif key == 'l':
                    if LED == False:
                       my_arm.serial.write((json.dumps({"T":114,"led":255})+'\n').encode())                                        #开启灯光
                       LED = True
                    else:
                       my_arm.serial.write((json.dumps({"T":114,"led":0})+'\n').encode())                                          #关闭灯光
                       LED = False
                elif key == 'w':
                    write_pose_flag = 1
                elif key == 'c':
                     os.system('echo " " > /home/epaicar/talos_ws/src/agrc_base_arm/config/pose_array.yaml')
                     write_count = 0
                elif key == 's':
                    my_arm.serial.write((json.dumps({"T":my_arm.degree_control,\
                                       "b":my_arm.M2_init_pose[0],\
                                       "s":my_arm.M2_init_pose[1],\
                                       "e":my_arm.M2_init_pose[2],\
                                       "h":my_arm.M2_init_pose[3],\
                                       "spd":my_arm.M2_speed,\
                                       "acc":my_arm.M2_acc}) + "\n").encode())
                    write_pose_flag = 2
                    if record_mode:
                      write_count += 1
                    else:
                      XYZT_count += 1
                elif key == 'q':
                    record_mode = not(record_mode)
                    print(COLOR_GREEN + "change the recording mode:" + str(record_mode) + STYLE_RESET)
                elif key == 'm':
                    print("Prepare to move to " + str(my_arm.seted_pose))
                elif key == 'k':
                    pose=my_arm.M2_init_pose
                    my_arm.reset_pose(pose)
                #修改设置机械臂角度模式
                # elif key == 'g':
                #     set_pose_mode = not(set_pose_mode)
                #     print(COLOR_GREEN + "change the set pose mode:" + str(set_pose_mode) + "开始设置角度，发布角度数据后再按 m 即可 " +STYLE_RESET)
                elif key == 'g':
                    print("x , y ,z:" + str(my_arm.M2_current_XYZt[0:3]) + " t:" + str(my_arm.M2_current_XYZt[3]))

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    except rospy.ROSInterruptException:
        my_arm.serial.close
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rospy.loginfo("arm_driver finished.")
