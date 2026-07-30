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

# 连续帧关联：新目标连续出现两帧才确认，短时漏检保留上一框。
CONFIRM_FRAMES = 2
LOST_GRACE_FRAMES = 3
MAX_TRACK_JUMP_PX = 60
BOX_FILTER_ALPHA = 0.35


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


def object_box(obj):
    return [
        float(obj.x),
        float(obj.y),
        float(obj.w),
        float(obj.h),
    ]


def box_center(box):
    return box[0] + box[2] * 0.5, box[1] + box[3] * 0.5


def center_distance(box_a, box_b):
    ax, ay = box_center(box_a)
    bx, by = box_center(box_b)
    dx = ax - bx
    dy = ay - by
    return (dx * dx + dy * dy) ** 0.5


def smooth_box(previous_box, current_box):
    alpha = BOX_FILTER_ALPHA
    return [
        previous_box[index]
        + alpha * (current_box[index] - previous_box[index])
        for index in range(4)
    ]


def select_candidate(candidates, reference_box, frame_width, frame_height):
    if not candidates:
        return None

    if reference_box is not None:
        nearby = [
            obj
            for obj in candidates
            if center_distance(object_box(obj), reference_box)
            <= MAX_TRACK_JUMP_PX
        ]
        if not nearby:
            return None
        candidates = nearby

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
        if reference_box is not None:
            score -= center_distance(
                object_box(obj),
                reference_box,
            ) * 0.8
        return score

    return max(candidates, key=candidate_score)


detector = nn.YOLO11(model=MODEL_PATH, dual_buff=False)
cam = camera.Camera(
    detector.input_width(),
    detector.input_height(),
    detector.input_format(),
)
disp = display.Display()
tracked_box = None
tracked_score = 0.0
pending_box = None
pending_count = 0
lost_count = 0

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
        tracked_box if tracked_box is not None else pending_box,
        img.width(),
        img.height(),
    )

    if selected is not None:
        selected_box = object_box(selected)
        lost_count = 0
        if tracked_box is not None:
            tracked_box = smooth_box(tracked_box, selected_box)
            tracked_score = selected.score
        else:
            if (
                pending_box is not None
                and center_distance(selected_box, pending_box)
                <= MAX_TRACK_JUMP_PX
            ):
                pending_box = smooth_box(pending_box, selected_box)
                pending_count += 1
            else:
                pending_box = selected_box
                pending_count = 1
            if pending_count >= CONFIRM_FRAMES:
                tracked_box = pending_box
                tracked_score = selected.score
                pending_box = None
                pending_count = 0
    else:
        pending_box = None
        pending_count = 0
        if tracked_box is not None:
            lost_count += 1
            if lost_count > LOST_GRACE_FRAMES:
                tracked_box = None
                tracked_score = 0.0

    img.draw_string(
        8,
        8,
        f"raw:{len(objects)} valid:{len(candidates)} lost:{lost_count}",
        color=image.COLOR_GREEN,
        scale=1.2,
    )

    if tracked_box is not None:
        box_x = int(round(tracked_box[0]))
        box_y = int(round(tracked_box[1]))
        box_w = int(round(tracked_box[2]))
        box_h = int(round(tracked_box[3]))
        img.draw_rect(
            box_x,
            box_y,
            box_w,
            box_h,
            color=image.COLOR_RED,
            thickness=3,
        )
        label = f"steel_ball: {tracked_score:.2f}"
        img.draw_string(
            box_x,
            max(0, box_y - 20),
            label,
            color=image.COLOR_RED,
            scale=1.0,
        )

    disp.show(img)
