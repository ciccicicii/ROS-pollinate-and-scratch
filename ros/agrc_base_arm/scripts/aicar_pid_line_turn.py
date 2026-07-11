#!/usr/bin/python
#coding:utf-8
import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from std_msgs.msg import Float32

# 初始化参数
min_output = 0.009  # 死区阈值
speed_x = 0.0
boost_factor = 0.0
Kp_yaw = 0.5
Kd_yaw = 1.5
Ki_yaw = 0.0

# 状态变量
# 在全局增加一个变量，保存上一次输出
last_angular_output = 0.0
want_yaw = 0.0
current_yaw = 0.0
base_yaw = 0.0  # 基准角度
turn_offsets = {}  # 存储不同转弯方向的偏移量
current_direction = 0 # 当前方向，0为直行，1为右转，-1为左转
stop_flag = 0
alignment_flag = 0
initialization_complete = False
initial_samples = []
INIT_SAMPLE_COUNT = 10  # 初始角度采样次数

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=50)

def normalize_angle(angle):
    """角度归一化到[-π, π]"""
    return math.atan2(math.sin(angle), math.cos(angle))

class PID:
    def __init__(self, kp_yaw, kd_yaw, ki_yaw):
        self.kp_yaw = kp_yaw
        self.kd_yaw = kd_yaw
        self.ki_yaw = ki_yaw
        self.previous_error_yaw = 0.0
        self.integral_yaw = 0.0
        self.integral_limit = 0.1

    def compute(self, current, target):
        error = normalize_angle(target - current)

        # 积分项
        self.integral_yaw += error
        self.integral_yaw = max(min(self.integral_yaw, self.integral_limit), -self.integral_limit)

        # 微分项
        derivative = error - self.previous_error_yaw
        self.previous_error_yaw = error

        # PID输出
        output = (
            self.kp_yaw * error +
            self.ki_yaw * self.integral_yaw +
            self.kd_yaw * derivative
        )
        return output

    def reset(self):
        """重置PID状态"""
        self.previous_error_yaw = 0.0
        self.integral_yaw = 0.0

def quaternion_to_euler(qw, qx, qy, qz):
    """四元数转欧拉角 (仅yaw)"""
    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy**2 + qz**2)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def ImuCallBack(msg):
    global current_yaw, base_yaw, initialization_complete, initial_samples

    # 转换四元数为欧拉角
    quaternion = (msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
    current_yaw = quaternion_to_euler(*quaternion)

    # 初始角度采样
    if not initialization_complete:
        initial_samples.append(current_yaw)
        if len(initial_samples) >= INIT_SAMPLE_COUNT:
            # 计算初始角度的平均值
            base_yaw = sum(initial_samples) / len(initial_samples)

            #初始角偏移量
            turn_offsets[0] = 0.0  # 直行方向
            turn_offsets[-1] = math.pi/2  # 左转90度
            turn_offsets[1] = -math.pi/2  # 右转90度
            #设置初始角度
            want_yaw = base_yaw + turn_offsets[current_direction]
            initialization_complete = True
            rospy.loginfo("初始角度采样完成: %.6f", base_yaw)

def stop_CallBack(msg):
    global stop_flag, alignment_flag, base_yaw, current_yaw

    prev_flag = stop_flag
    stop_flag = msg.data

    # 从停止到启动的转换
    if prev_flag == 0 and stop_flag == 1:
        # 更新基准角度为当前角度
        # base_yaw = current_yaw
        # want_yaw = base_yaw
        pid.reset()  # 重置PID状态
        rospy.loginfo("新的基准角度设置为: %.6f", base_yaw)

    # 从运行到停止的转换
    elif prev_flag != 0 and stop_flag == 0:
        alignment_flag = 1  # 标记需要进行角度对齐
def direction_CallBack(msg):
    """处理方向切换命令"""
    global current_direction, want_yaw, base_yaw, turn_offsets

    new_direction = msg.data
    if new_direction in turn_offsets:
        current_direction = new_direction
        want_yaw = normalize_angle(base_yaw + turn_offsets[current_direction])
        rospy.loginfo("切换方向到: %s, 目标角度: %.6f", current_direction, want_yaw)
    else:
        rospy.logwarn("未知方向: %s", new_direction)
def dotime(event):
    global want_yaw, current_yaw, base_yaw, stop_flag, alignment_flag,last_angular_output

    if not initialization_complete:
        return  # 等待初始化完成

    msg = Twist()

    # 运行状态（前进或后退）
    if stop_flag == 1 or stop_flag == -1:
        # 更新目标角度为基准角度
        want_yaw = normalize_angle(base_yaw + turn_offsets[current_direction])

        # 计算PID输出
        pid_output = pid.compute(current_yaw, want_yaw)

        # 角速度限幅
        max_angular = 0.5 if stop_flag == 1 else 1.0
        pid_output = max(min(pid_output, max_angular), -max_angular)

        # 死区补偿
        if abs(pid_output) < min_output:
            target_output = 0.0
        else:
            target_output = pid_output
        # 限制每次角速度变化的最大步长（ramp）
        max_step = 0.08  # 每次最多变化 0.02 rad/s
        if target_output > last_angular_output + max_step:
            smoothed_output = last_angular_output + max_step
        elif target_output < last_angular_output - max_step:
            smoothed_output = last_angular_output - max_step
        else:
            smoothed_output = target_output

        last_angular_output = smoothed_output
        msg.angular.z = smoothed_output
        # 线速度设置
        msg.linear.x = speed_x if stop_flag == 1 else -speed_x
    elif stop_flag == 3:
        # 休眠状态，不发布任何速度消息
        return
    # 停止状态
    elif stop_flag == 2:
        #转弯过程
        target_yaw = normalize_angle(base_yaw + turn_offsets[current_direction])
        yaw_error = normalize_angle(target_yaw - current_yaw)
        if abs(yaw_error) > 0.02:  # 约1度
            # 增大转弯角速度限幅
            pid_output = pid.compute(current_yaw, target_yaw)
            pid_output = max(min(pid_output, 3), -3)  # 角速度上限调大
            msg.angular.z = pid_output
            msg.linear.x = 0.0
        else:
            # 转弯完成，更新基准角度
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            pid.reset()
    else:
        # 角度对齐过程
        if alignment_flag:
            target_yaw = normalize_angle(base_yaw + turn_offsets[current_direction])
            yaw_error = normalize_angle(target_yaw - current_yaw)

            if abs(yaw_error) > 0.02:  # 约1度
                pid_output = pid.compute(current_yaw, target_yaw)
                pid_output = max(min(pid_output, 0.5), -0.5)
                msg.angular.z = pid_output
                msg.linear.x = 0.0
            else:
                # 对齐完成
                msg.angular.z = 0.0
                msg.linear.x = 0.0
                alignment_flag = 0
                pid.reset()  # 重置PID状态
                rospy.loginfo("角度对齐完成")
        else:
            # 保持停止状态
            msg.angular.z = 0.0
            msg.linear.x = 0.0

    pub.publish(msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ucar_pid_line_node', anonymous=True)

        # 从参数服务器获取参数
        speed_x = rospy.get_param("~speed_x", 0.0)
        boost_factor = rospy.get_param("~boost_factor", 0.0)
        Kp_yaw = rospy.get_param("~Kp_yaw", 0.8)
        Kd_yaw = rospy.get_param("~Kd_yaw", 0.0)
        Ki_yaw = rospy.get_param("~Ki_yaw", 0.0)
        min_output = 0.009

        # 创建PID控制器
        pid = PID(Kp_yaw, Kd_yaw, Ki_yaw)

        # 订阅话题
        rospy.Subscriber("/imu_data", Imu, ImuCallBack)
        rospy.Subscriber("/stop_flag", Int8, stop_CallBack)
        rospy.Subscriber("/direction", Int8, direction_CallBack)  # 新增方向切换话题
        # 日志输出参数信息
        rospy.loginfo("PID参数: Kp=%.4f, Kd=%.4f, Ki=%.4f", Kp_yaw, Kd_yaw, Ki_yaw)
        rospy.loginfo("速度: %.2f m/s, 死区: %.3f", speed_x, min_output)

        # 启动定时器
        rospy.Timer(rospy.Duration(0.05), dotime)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("节点已终止")