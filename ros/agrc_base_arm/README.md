# agrc_base_arm

该目录保留项目实际使用的 ROS 脚本、机械臂轨迹参数和 launch 文件。

脚本依赖 ROS1、`rospy`、OpenCV、pyzbar、pyserial、move_base 及对应机器人硬件驱动。由于公开仓库不包含完整车端工作空间，launch 文件用于说明启动关系；运行前还需要安装底盘、IMU、雷达和视觉桥接包。
