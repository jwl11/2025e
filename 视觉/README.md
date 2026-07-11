# MaixCAM2 矩形识别

## 文件

- `main.py`：识别高对比度矩形靶框，画出四角和中心，并输出中心像素误差。
- `main_ai.py`：使用官方 E 题 AI 模型，适合复杂背景、局部遮挡和部分出画。

## 运行

1. 将整个“视觉”文件夹作为工程用 MaixVision 打开。
2. 连接 MaixCAM2。
3. 运行 `main.py`。
4. 将 A4 靶纸黑框完整放入画面，观察红色四边形和终端输出。

若需要抗遮挡，按 `model/README.md` 安装模型后运行 `main_ai.py`。输出中 `source=ai` 是真实检测，`source=predict` 是短时运动预测。

启动时终端必须出现 `RECT_DETECTOR_VERSION=opencv_v5_edge_track`。如果没有出现，说明 MaixVision 运行的仍是开发板上的旧文件，需要重新保存并点击“运行文件”，或重新把整个工程同步到开发板。

输出示例：

```text
RECT found=1 cx=328 cy=235 ex=8 ey=-5 area=45231 fps=58.4
```

字段说明：

- `found`：1 表示识别到目标，0 表示未识别。
- `cx/cy`：目标矩形中心坐标。
- `ex/ey`：目标中心相对画面中心的像素误差。
- `area`：目标四边形面积。
- `fps`：当前运行帧率。

## 首次调参

主要修改 `main.py` 顶部参数：

- 漏检：适当减小 `CANNY_LOW_THRESHOLD` 和 `CANNY_HIGH_THRESHOLD`。
- 误检：增大 Canny 阈值或 `MIN_RECT_AREA`。
- 白纸偏暗导致漏检：减小 `WHITE_VALUE_MIN`，例如从 115 调到 100。
- 浅色背景被误识别：减小 `WHITE_SATURATION_MAX`，并尽量使用深色、无反光背景。
- 远距离靶框被过滤：减小 `MIN_RECT_AREA` 和 `MIN_EDGE_LENGTH`。
- 中心附近来回抖动：增大 `CENTER_DEAD_ZONE_PX`。
- 目标中心仍抖动：减小 `CENTER_SMOOTH_ALPHA`；跟随太慢则增大该值。
- 靠边短时丢失：可适当增大 `LOST_HOLD_FRAMES`，但不宜过大，否则会长时间使用旧坐标。

调试时保证：黑框四边都在画面内、镜头已对焦、靶纸没有强烈反光。矩形基础识别稳定后，再加入透视矫正、圆心识别和 UART 数据帧。

## 版本说明

程序使用 MaixPy v4 的 `camera.Camera()`、`display.Display()`、`image.image2cv()`，再使用 OpenCV 完成边缘、轮廓和四边形检测，不适用于旧版 K210/MaixPy v1 的 `sensor.snapshot()` API。

默认识别分辨率为 320×240。最初的 `img.find_rects()` 方案在部分 MaixCAM2 系统镜像中即使使用该分辨率，仍可能报 `Out of fast Frame Buffer Stack Memory`，因此当前版本已改为官方 E 题示例同类的 OpenCV 轮廓方案，不再调用 `find_rects()`。
