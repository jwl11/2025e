# MaixCAM2 E题黑框模型

模型来源：MaixHub 1370。

- `best.mud`：MaixPy模型入口，`model_type=yolo11`、`labels=Kuang`。
- `best_npu.axmodel`：MaixCAM2 AX630C NPU2模型。
- `best_vnpu.axmodel`：配套模型文件。

程序通过相对路径 `model/best.mud` 加载；也可以把三个文件一起放到设备的 `/root/models/`。

