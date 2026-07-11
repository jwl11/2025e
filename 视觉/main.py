"""
MaixCAM2 / MaixPy v4 矩形靶框识别

用途：
1. 识别画面中的高对比度四边形/矩形；
2. 从候选矩形中选出面积最大且形状合理的目标；
3. 输出矩形中心相对画面中心的像素误差，为后续 MSPM0 云台控制提供输入。

依据：
- sipeed/MaixPy examples/vision/image_basic/find_rects.py
- sipeed/MaixPy projects/demo_diansai_2025_E_circle_track
"""

from maix import app, camera, display, image, time
import cv2
import numpy as np


# ============================== 可调参数 ==============================
# MaixPy 的 find_rects() 会使用 Fast Frame Buffer。
# 640x480 RGB 在部分 MaixCAM2 镜像/屏幕配置下会耗尽该内存，
# 官方基础示例采用 320x240，因此这里默认使用安全分辨率。
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_FPS = 60

# OpenCV 边缘检测阈值。环境较暗或黑框较浅时可适当减小。
CANNY_LOW_THRESHOLD = 45
CANNY_HIGH_THRESHOLD = 135

# 多边形近似精度，越大越容易把轮廓近似为四边形。
APPROX_EPSILON_RATIO = 0.025

# 排除过小候选框。
MIN_RECT_AREA = 1200
MIN_EDGE_LENGTH = 18

# 透视会改变外接框宽高比，因此这里只做宽松筛选。
MIN_BOUNDING_RATIO = 0.45
MAX_BOUNDING_RATIO = 2.20

# 中心误差小于该值时按 0 处理，避免控制系统在中心附近抖动。
CENTER_DEAD_ZONE_PX = 2

# 每隔多少帧向终端输出一次结果。
PRINT_INTERVAL = 10
# ====================================================================


def polygon_area(corners):
    """用鞋带公式计算四边形面积。"""
    area2 = 0
    count = len(corners)
    for i in range(count):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % count]
        area2 += x1 * y2 - x2 * y1
    return abs(area2) / 2.0


def edge_length_squared(point_a, point_b):
    dx = point_a[0] - point_b[0]
    dy = point_a[1] - point_b[1]
    return dx * dx + dy * dy


def rectangle_center(corners):
    """四角均值比外接矩形中心更适合有透视变形的目标。"""
    center_x = int(sum(point[0] for point in corners) / 4)
    center_y = int(sum(point[1] for point in corners) / 4)
    return center_x, center_y


def detect_target(img, debug=False):
    """使用 OpenCV 轮廓法检测面积最大的有效四边形。"""
    if debug:
        print("stage=image_to_cv")
    img_cv = image.image2cv(img, False, False)
    if debug:
        print("stage=grayscale")
    gray = cv2.cvtColor(img_cv, cv2.COLOR_RGB2GRAY)
    if debug:
        print("stage=gaussian")
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    if debug:
        print("stage=canny")
    edges = cv2.Canny(blurred, CANNY_LOW_THRESHOLD, CANNY_HIGH_THRESHOLD)

    # 闭运算连接黑框上可能断开的边缘。
    kernel = np.ones((3, 3), np.uint8)
    if debug:
        print("stage=morphology")
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=1)
    if debug:
        print("stage=find_contours")
    contours, _ = cv2.findContours(
        edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    best = None
    best_area = 0
    min_edge_squared = MIN_EDGE_LENGTH * MIN_EDGE_LENGTH

    for contour in contours:
        contour_area = cv2.contourArea(contour)
        if contour_area < MIN_RECT_AREA:
            continue

        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(
            contour, APPROX_EPSILON_RATIO * perimeter, True
        )
        if len(approx) != 4 or not cv2.isContourConvex(approx):
            continue

        corners = [tuple(int(v) for v in point[0]) for point in approx]
        area = polygon_area(corners)
        if area < MIN_RECT_AREA:
            continue

        if any(
            edge_length_squared(corners[i], corners[(i + 1) % 4])
            < min_edge_squared
            for i in range(4)
        ):
            continue

        x, y, width, height = cv2.boundingRect(approx)
        if width <= 0 or height <= 0:
            continue

        ratio = width / height
        if ratio < MIN_BOUNDING_RATIO or ratio > MAX_BOUNDING_RATIO:
            continue

        if area > best_area:
            best = {
                "corners": corners,
                "rect": (x, y, width, height),
                "area": area,
            }
            best_area = area

    return best, best_area


def draw_quadrilateral(img, corners, color, thickness=3):
    for i in range(4):
        start = corners[i]
        end = corners[(i + 1) % 4]
        img.draw_line(
            start[0], start[1], end[0], end[1], color, thickness=thickness
        )


def apply_dead_zone(value):
    return 0 if abs(value) <= CENTER_DEAD_ZONE_PX else value


def main():
    print("RECT_DETECTOR_VERSION=opencv_v3")
    print("stage=init_camera")
    cam = camera.Camera(CAMERA_WIDTH, CAMERA_HEIGHT, fps=CAMERA_FPS)
    print("stage=init_display")
    disp = display.Display()
    print("stage=ready width=%d height=%d" % (CAMERA_WIDTH, CAMERA_HEIGHT))

    screen_center_x = CAMERA_WIDTH // 2
    screen_center_y = CAMERA_HEIGHT // 2
    frame_count = 0

    while not app.need_exit():
        print_stage = frame_count == 0
        if print_stage:
            print("stage=read_camera")
        img = cam.read()
        if print_stage:
            print("stage=detect_target")
        target, target_area = detect_target(img, debug=print_stage)
        if print_stage:
            print("stage=draw_result")

        # 画出画面中心。
        img.draw_circle(
            screen_center_x,
            screen_center_y,
            5,
            image.COLOR_GREEN,
            thickness=2,
        )

        found = target is not None
        center_x = -1
        center_y = -1
        error_x = 0
        error_y = 0

        if found:
            corners = target["corners"]
            center_x, center_y = rectangle_center(corners)
            error_x = apply_dead_zone(center_x - screen_center_x)
            error_y = apply_dead_zone(center_y - screen_center_y)

            draw_quadrilateral(img, corners, image.COLOR_RED, thickness=3)
            img.draw_circle(center_x, center_y, 6, image.COLOR_RED, thickness=-1)
            img.draw_line(
                screen_center_x,
                screen_center_y,
                center_x,
                center_y,
                image.COLOR_BLUE,
                thickness=2,
            )

            x, y, _, _ = target["rect"]
            img.draw_string(
                x,
                max(0, y - 22),
                "target ex:%d ey:%d" % (error_x, error_y),
                image.COLOR_RED,
            )
        else:
            img.draw_string(10, 10, "rectangle not found", image.COLOR_ORANGE)

        if frame_count % PRINT_INTERVAL == 0:
            # 后续接 UART 时，可以把以下字段封装为定长二进制帧：
            # found, center_x, center_y, error_x, error_y, area
            print(
                "RECT found=%d cx=%d cy=%d ex=%d ey=%d area=%d fps=%.1f"
                % (
                    1 if found else 0,
                    center_x,
                    center_y,
                    error_x,
                    error_y,
                    int(target_area),
                    time.fps(),
                )
            )

        frame_count += 1
        if print_stage:
            print("stage=display_result")
        disp.show(img)
        if print_stage:
            print("stage=first_frame_complete")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback

        print(traceback.format_exc())
        while not app.need_exit():
            time.sleep_ms(100)
