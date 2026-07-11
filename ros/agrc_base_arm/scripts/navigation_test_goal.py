#!/usr/bin/python
#coding:utf-8
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# 多个目标点（位置 + 方向）
goal_list = [
    {'x': 0.88, 'y': 0, 'yaw': 0.0},
    {'x': 1.4, 'y': 0, 'yaw': 0.0},
    {'x': 1.84, 'y': -0.02, 'yaw': 0},
    {'x': 2.4, 'y': -0.02, 'yaw': 0},
    {'x': 0, 'y': 0, 'yaw': -1.59}
]

def send_goal(x, y, yaw):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    # 转换 yaw 为 quaternion
    from tf.transformations import quaternion_from_euler
    q = quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Goal reached")

if __name__ == '__main__':
    rospy.init_node('multi_goal_sender')

    for goal in goal_list:
        send_goal(goal['x'], goal['y'], goal['yaw'])
        rospy.sleep(2.0)  # 到达后停顿2秒
