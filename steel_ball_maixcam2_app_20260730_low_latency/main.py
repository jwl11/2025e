import os

from maix import app, camera, display, image, nn


MODEL_PATH = "model/steel_ball_yolo11n.mud"
if not os.path.exists(MODEL_PATH):
    MODEL_PATH = "/root/models/steel_ball_yolo11n.mud"

# 0.025 会放进大量螺丝孔和螺母误检，现场模型先从 0.20 起测。
BALL_CLASS_ID = 0
CONFIDENCE = 0.20
IOU_THRESHOLD = 0.45

# 960x544 原画中钢球框的合理范围。过滤小孔和大面积反光物。
MIN_DIAMETER = 16
MAX_DIAMETER = 45
MAX_ASPECT_RATIO = 1.35
EXPECTED_DIAMETER = 24

# 根据当前安装画面标定的绿色水管中心线。
PIPE_LEFT_X_RATIO = 0.06
PIPE_LEFT_Y_RATIO = 0.59
PIPE_RIGHT_X_RATIO = 0.86
PIPE_RIGHT_Y_RATIO = 0.46
PIPE_HALF_BAND_PX = 32

def pipe_center_y(center_x, frame_width, frame_height):
    left_x = frame_width * PIPE_LEFT_X_RATIO
    left_y = frame_height * PIPE_LEFT_Y_RATIO
    right_x = frame_width * PIPE_RIGHT_X_RATIO
    right_y = frame_height * PIPE_RIGHT_Y_RATIO
    ratio = (center_x - left_x) / max(1.0, right_x - left_x)
    return left_y + ratio * (right_y - left_y)


def is_valid_ball_candidate(obj, frame_width, frame_height):
    if obj.class_id != BALL_CLASS_ID:
        return False

    short_side = min(obj.w, obj.h)
    long_side = max(obj.w, obj.h)
    if short_side < MIN_DIAMETER or long_side > MAX_DIAMETER:
        return False
    if long_side / max(1, short_side) > MAX_ASPECT_RATIO:
        return False

    center_x = obj.x + obj.w * 0.5
    center_y = obj.y + obj.h * 0.5
    left_x = frame_width * PIPE_LEFT_X_RATIO
    right_x = frame_width * PIPE_RIGHT_X_RATIO
    if center_x < left_x or center_x > right_x:
        return False
    return (
        abs(
            center_y
            - pipe_center_y(center_x, frame_width, frame_height)
        )
        <= PIPE_HALF_BAND_PX
    )


def select_candidate(candidates, frame_width, frame_height):
    if not candidates:
        return None

    def candidate_score(obj):
        center_x = obj.x + obj.w * 0.5
        center_y = obj.y + obj.h * 0.5
        pipe_error = abs(
            center_y
            - pipe_center_y(center_x, frame_width, frame_height)
        )
        diameter = (obj.w + obj.h) * 0.5
        score = (
            obj.score * 100.0
            - pipe_error * 0.5
            - abs(diameter - EXPECTED_DIAMETER) * 0.4
        )
        return score

    return max(candidates, key=candidate_score)


detector = nn.YOLO11(model=MODEL_PATH, dual_buff=False)
cam = camera.Camera(
    detector.input_width(),
    detector.input_height(),
    detector.input_format(),
)
disp = display.Display()

while not app.need_exit():
    img = cam.read()
    objects = detector.detect(
        img,
        conf_th=CONFIDENCE,
        iou_th=IOU_THRESHOLD,
    )

    candidates = [
        obj
        for obj in objects
        if is_valid_ball_candidate(
            obj,
            img.width(),
            img.height(),
        )
    ]
    selected = select_candidate(
        candidates,
        img.width(),
        img.height(),
    )

    img.draw_string(
        8,
        8,
        f"raw:{len(objects)} valid:{len(candidates)}",
        color=image.COLOR_GREEN,
        scale=1.2,
    )

    if selected is not None:
        box_x = int(selected.x)
        box_y = int(selected.y)
        box_w = int(selected.w)
        box_h = int(selected.h)
        img.draw_rect(
            box_x,
            box_y,
            box_w,
            box_h,
            color=image.COLOR_RED,
            thickness=3,
        )
        label = f"steel_ball: {selected.score:.2f}"
        img.draw_string(
            box_x,
            max(0, box_y - 20),
            label,
            color=image.COLOR_RED,
            scale=1.0,
        )

    disp.show(img)
