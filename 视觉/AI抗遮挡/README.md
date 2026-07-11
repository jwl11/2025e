# MaixCAM2 E题AI黑框识别

本工程已经包含MaixHub 1370的YOLO11模型与检测程序。

在MaixVision中打开整个“AI抗遮挡”文件夹，选择“运行工程”，不要只运行单个文件；`app.yaml`会将`main.py`和`model/`一起同步到设备。

设备要求MaixPy 4.7.0或更高版本，并需要提供`nn.YOLO11`接口。

启动成功时终端会显示：

```text
RECT_DETECTOR_VERSION=yolo11_1370_v1
```

