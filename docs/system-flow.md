# 系统执行流程

## 启动与任务执行

```mermaid
sequenceDiagram
    participant QR as 二维码节点
    participant Main as new_scra.py
    participant Vision as YOLOv5 / DeepStream
    participant IMU as aicar_pid_line_turn.py
    participant Base as 移动底盘
    participant Arm as new_arm.py

    QR->>Main: 任务点序列
    Main->>IMU: stop_flag + direction
    IMU->>Base: cmd_vel
    Main->>Arm: 识别姿态动作编号
    Vision->>Main: 类别和目标框信息
    alt 成熟目标
        Main->>Arm: 采摘动作编号
    else 腐烂目标
        Main->>Arm: 移除动作编号
    else 未成熟目标
        Main->>Arm: 复位动作编号
    end
    Main->>IMU: 下一路段运动指令
```

## C 区拓扑点位

任务点用 `(road, row)` 表示。路径执行只关心当前点和目标点之间的拓扑关系：

- `road` 相同：根据 `row` 差值前进或后退；
- `road` 不同：退出当前通道，执行换道动作，再进入目标通道；
- 共享点位：避免重复执行不必要的换道。

这种方法针对固定比赛场地，不依赖完整二维地图，但需要提前标定每段运动时间。
