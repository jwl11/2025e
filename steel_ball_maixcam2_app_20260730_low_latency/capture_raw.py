import os

from maix import app, camera, display, image, nn


MODEL_PATH = "model/steel_ball_yolo11n.mud"
if not os.path.exists(MODEL_PATH):
    MODEL_PATH = "/root/models/steel_ball_yolo11n.mud"

SAVE_FRAMES = (30, 60, 90, 120, 150)


detector = nn.YOLO11(model=MODEL_PATH, dual_buff=False)
cam = camera.Camera(
    detector.input_width(),
    detector.input_height(),
    detector.input_format(),
)
disp = display.Display()
frame_index = 0
saved_count = 0

while not app.need_exit():
    img = cam.read()
    frame_index += 1

    if frame_index in SAVE_FRAMES:
        saved_count += 1
        img.save(f"steel_ball_raw_{saved_count:02d}.jpeg", quality=95)

    img.draw_string(
        8,
        8,
        f"raw saved: {saved_count}/{len(SAVE_FRAMES)}",
        color=image.COLOR_GREEN,
        scale=1.2,
    )
    disp.show(img)
