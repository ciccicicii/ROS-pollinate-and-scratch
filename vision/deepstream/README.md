# DeepStream / TensorRT 部署材料

部署链路：

1. YOLOv5 训练得到权重；
2. 将模型转换为 DeepStream-Yolo 使用的 `.cfg/.wts`；
3. 通过 `config_infer_primary_yolov5.txt` 配置类别数、标签、阈值和 TensorRT engine；
4. DeepStream 自定义 parser 解析检测结果；
5. ROS 视觉桥接节点读取最新结果并发布 `/yolov5_sign`。

`integration/nvdsparsebbox_Yolo.cpp` 基于 DeepStream-Yolo 解析器修改，增加了将过滤后类别结果写入中间文件的逻辑。该文本文件桥接方式便于现场调试，但不是严格实时通信；后续可改为直接从 DeepStream 发布 ROS 消息。

模型权重、TensorRT engine、编译生成的 `.so/.o` 和运行时 `internal_memory.txt` 均未上传。
