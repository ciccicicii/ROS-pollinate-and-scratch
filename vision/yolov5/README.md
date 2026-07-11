# YOLOv5 训练材料

该目录只保留本项目相关的训练配置和验证结果，不重复上传完整 YOLOv5 第三方源码及模型权重。

## 果蔬模型

- 基础模型：YOLOv5s
- 输入尺寸：640
- batch size：16
- epoch：100
- 优化器：SGD
- 类别数：13
- 数据增强：HSV、平移、缩放、水平翻转、Mosaic、MixUp

最后一轮本地验证集指标：Precision `0.97995`、Recall `0.99327`、mAP@0.5 `0.99077`、mAP@0.5:0.95 `0.85486`。

`results/harvest` 中包含训练曲线、PR 曲线、混淆矩阵、验证预测图和原始 `results.csv`。

## 未上传内容

`.pt`、`.wts`、`.engine` 等权重和设备相关推理文件未上传。完整训练框架请参考 YOLOv5 官方项目，第三方许可见仓库根目录 `THIRD_PARTY.md`。
