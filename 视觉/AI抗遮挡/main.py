"""MaixCAM2 AI 黑框检测与短时遮挡预测。

模型来源：MaixHub 1370，MaixCAM2 YOLO11 E题黑框模型。
AI 检测负责复杂背景、部分遮挡和局部出画；短时完全丢失时仅预测，
预测结果不属于真实视觉观测。
"""

from maix import app, camera, display, image, nn, time
import os


MODEL_PATHS = (
    "/root/models/best.mud",
    "model/best.mud",
)
CONFIDENCE_THRESHOLD = 0.50
IOU_THRESHOLD = 0.45
SMOOTH_ALPHA = 0.30
VELOCITY_ALPHA = 0.25
VELOCITY_DECAY = 0.92
LOST_PREDICT_FRAMES = 30
PRINT_INTERVAL = 10


def find_model():
    for path in MODEL_PATHS:
        if os.path.exists(path):
            return path
    raise RuntimeError(
        "model missing: download MaixHub model zoo 1370, then copy the full "
        "model package to /root/models or this project's model directory"
    )


def main():
    print("RECT_DETECTOR_VERSION=yolo11_1370_v1")
    model_path = find_model()
    detector = nn.YOLO11(model=model_path, dual_buff=True)
    camera_width = detector.input_width()
    camera_height = detector.input_height()
    cam = camera.Camera(
        camera_width,
        camera_height,
        detector.input_format(),
        buff_num=1,
    )
    disp = display.Display()

    image_center_x = camera_width // 2
    image_center_y = camera_height // 2

    smooth_x = None
    smooth_y = None
    last_raw_x = None
    last_raw_y = None
    velocity_x = 0.0
    velocity_y = 0.0
    lost_frames = LOST_PREDICT_FRAMES + 1
    frame_count = 0

    while not app.need_exit():
        frame = cam.read()
        objects = detector.detect(
            frame,
            conf_th=CONFIDENCE_THRESHOLD,
            iou_th=IOU_THRESHOLD,
        )

        target = None
        target_area = 0
        for obj in objects:
            area = obj.w * obj.h
            if area > target_area:
                target = obj
                target_area = area

        detected = target is not None
        source = "none"

        if detected:
            x = int(target.x)
            y = int(target.y)
            width = int(target.w)
            height = int(target.h)
            raw_x = x + width // 2
            raw_y = y + height // 2

            if smooth_x is None:
                smooth_x = float(raw_x)
                smooth_y = float(raw_y)
            else:
                if last_raw_x is not None:
                    velocity_x += VELOCITY_ALPHA * (
                        (raw_x - last_raw_x) - velocity_x
                    )
                    velocity_y += VELOCITY_ALPHA * (
                        (raw_y - last_raw_y) - velocity_y
                    )
                smooth_x += SMOOTH_ALPHA * (raw_x - smooth_x)
                smooth_y += SMOOTH_ALPHA * (raw_y - smooth_y)

            last_raw_x = raw_x
            last_raw_y = raw_y
            lost_frames = 0
            source = "ai"
            frame.draw_rect(x, y, width, height, image.COLOR_RED, thickness=3)
        else:
            lost_frames += 1
            if smooth_x is not None and lost_frames <= LOST_PREDICT_FRAMES:
                smooth_x += velocity_x
                smooth_y += velocity_y
                velocity_x *= VELOCITY_DECAY
                velocity_y *= VELOCITY_DECAY
                smooth_x = max(0.0, min(camera_width - 1.0, smooth_x))
                smooth_y = max(0.0, min(camera_height - 1.0, smooth_y))
                source = "predict"

        valid = source != "none"
        center_x = int(round(smooth_x)) if valid else -1
        center_y = int(round(smooth_y)) if valid else -1
        error_x = center_x - image_center_x if valid else 0
        error_y = center_y - image_center_y if valid else 0

        # 固定画面中心。
        frame.draw_line(
            image_center_x - 8,
            image_center_y,
            image_center_x + 8,
            image_center_y,
            image.COLOR_GREEN,
            thickness=2,
        )
        frame.draw_line(
            image_center_x,
            image_center_y - 8,
            image_center_x,
            image_center_y + 8,
            image.COLOR_GREEN,
            thickness=2,
        )

        if valid:
            color = image.COLOR_RED if source == "ai" else image.COLOR_ORANGE
            frame.draw_circle(center_x, center_y, 6, color, thickness=2)
            frame.draw_string(
                5, 5, "%s ex:%d ey:%d" % (source, error_x, error_y), color
            )
        else:
            frame.draw_string(5, 5, "target lost", image.COLOR_ORANGE)

        if frame_count % PRINT_INTERVAL == 0:
            print(
                "RECT valid=%d detected=%d source=%s cx=%d cy=%d "
                "ex=%d ey=%d fps=%.1f"
                % (
                    1 if valid else 0,
                    1 if detected else 0,
                    source,
                    center_x,
                    center_y,
                    error_x,
                    error_y,
                    time.fps(),
                )
            )

        frame_count += 1
        disp.show(frame)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback

        print(traceback.format_exc())
        while not app.need_exit():
            time.sleep_ms(100)
