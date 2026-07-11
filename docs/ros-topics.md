# ROS 话题说明

| 话题 | 消息类型 | 发布方 | 订阅方 | 用途 |
| --- | --- | --- | --- | --- |
| `/imu_data` | `sensor_msgs/Imu` | IMU 驱动 | `aicar_pid_line_turn.py` | 获取实时航向 |
| `/stop_flag` | `std_msgs/Int8` | `new_scra.py` | 航向控制 | 前进、后退、停止和转向状态 |
| `/direction` | `std_msgs/Int8` | `new_scra.py` | 航向控制 | 相对于初始航向的目标方向 |
| `/cmd_vel` | `geometry_msgs/Twist` | 航向控制 | 底盘驱动 | 线速度和角速度 |
| `/arm_cmd` | `std_msgs/Int32` | `new_scra.py` | `new_arm.py` | 机械臂预设动作编号 |
| `/arm_coord` | `std_msgs/Float32MultiArray` | 调试节点 | `new_arm.py` | 机械臂末端坐标控制 |
| `/arm_pose` | `std_msgs/Float32MultiArray` | `new_arm.py` | 调试工具 | 当前关节位置 |
| `/yolov5_sign` | `std_msgs/Int32MultiArray` | 视觉桥接 | `new_scra.py` | 类别和目标框信息 |
| `/scan` | `sensor_msgs/LaserScan` | 雷达驱动 | `get_scan_data.py` | 指定方向障碍距离 |
| `/fruit_array` | `std_msgs/Int32MultiArray` | `Qr_detect.py` | 现场任务节点 | 二维码解析结果 |
| `/Qr_scan` | `std_msgs/Int32MultiArray` | 另一版二维码节点 | `new_scra.py` | C 区任务点序列 |

## `stop_flag` 约定

| 值 | 含义 |
| ---: | --- |
| `1` | 前进并保持当前目标航向 |
| `-1` | 后退并保持当前目标航向 |
| `0` | 停止，并在需要时继续做航向对齐 |
| `2` | 原地转向到 `direction` 指定的方向 |
| `3` | 暂停速度发布（部分调试流程使用） |

## 版本说明

公开代码保留了比赛调试过程中不同节点的接口。二维码话题和视觉消息格式存在版本差异，部署时需要统一发布方与订阅方；这里不为了展示而伪造一个未在实车验证过的接口。
