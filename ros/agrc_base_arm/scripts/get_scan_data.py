#!/usr/bin/python
# coding=utf-8

import rospy
from sensor_msgs.msg import LaserScan

PI = 3.14159
class GET_SCAN:

    def get_laser_min_dis(self, scan_data):
        # 获取角度的索引，例如角度为30度
        right_dis = 0
        left_dis = 0
        left_desired_degrees = self.left_dis_angle
        right_desired_degrees = self.right_dis_angle
        left_desired_angle_rad = left_desired_degrees / 180.0 * PI
        right_desired_angle_rad = right_desired_degrees / 180.0 * PI
        left_angle_index = int((left_desired_angle_rad - scan_data.angle_min) / scan_data.angle_increment)
        right_angle_index = int((right_desired_angle_rad - scan_data.angle_min) / scan_data.angle_increment)

        rospy.loginfo("angle_min = %f, angle_max = %f, angle_increment = %f", scan_data.angle_min, scan_data.angle_max, scan_data.angle_increment)
        rospy.loginfo("left_angle_index = %d, right_angle_index = %d",left_angle_index, right_angle_index)

        if 0 <= left_angle_index < len(scan_data.ranges):
            left_dis = min(scan_data.ranges[left_angle_index:left_angle_index+2])
            rospy.loginfo("Left Dis at %f degrees: %f meters", left_desired_degrees, left_dis)
        else:
            rospy.logwarn("Left Desired angle is out of range")

        if 0 <= left_angle_index < len(scan_data.ranges):
            right_dis = min(scan_data.ranges[right_angle_index:right_angle_index+2])
            rospy.loginfo("Right Dis at %f degrees: %f meters", right_desired_degrees, right_dis)
        else:
            rospy.logwarn("Left Desired angle is out of range")

        self.min_dis = [left_dis, right_dis]

    def __init__(self):


        # 订阅雷达数据，获取左右两侧障碍物的最小距离
        self.left_dis_angle = 90
        self.right_dis_angle = 270
        self.min_dis = [10,10]
        rospy.Subscriber('/scan', LaserScan, self.get_laser_min_dis)

#main function
if __name__=="__main__":
    try:
        rospy.init_node('laser_distance_at_angle_node')
        rospy.loginfo('get_scan_node start...')
        get_scan = GET_SCAN()
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")
