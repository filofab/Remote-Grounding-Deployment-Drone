import cv2
import numpy as np
import time
import ArducamDepthCamera as ac
from pymavlink import mavutil

from config import *
from util import apply_flip

_last_yaw_time = 0.0
_tempo_in_tolleranza = None


def condition_yaw(vehicle, heading, direction=1):
    dir_val = 1 if direction == 1 else 0
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading,
        0,
        dir_val,
        1,
        0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def measure_distance_vertical(depth, confidence, x_center):
    h, w = depth.shape
    x_min = max(0, x_center - ROI_WIDTH_PX // 2)
    x_max = min(w, x_center + ROI_WIDTH_PX // 2)

    roi_d = depth[:, x_min:x_max]
    roi_c = confidence[:, x_min:x_max]

    valid = (
        (roi_d >= MIN_DISTANCE_MM) &
        (roi_d <= MAX_DISTANCE_MM) &
        (roi_c >= CONFIDENCE_THRESHOLD)
    )

    vals = roi_d[valid]
    if vals.size < 50:
        return None

    return int(np.percentile(vals, PERCENTILE_DISTANCE))


def passo_1_step(vehicle, cam, img_out):
    global _last_yaw_time, _tempo_in_tolleranza

    frame = cam.requestFrame(0)
    if frame is None or not isinstance(frame, ac.DepthData):
        return False

    depth = apply_flip(frame.depth_data)
    conf = apply_flip(frame.confidence_data)

    h, w = depth.shape
    cx = w // 2
    xl = cx - VERTICAL_LINE_SPACING_PX // 2
    xr = cx + VERTICAL_LINE_SPACING_PX // 2

    dl = measure_distance_vertical(depth, conf, xl)
    dr = measure_distance_vertical(depth, conf, xr)

    now = time.monotonic()

    if dl is not None and dr is not None:
        diff = dl - dr

        if abs(diff) <= SOGLIA_TOLLERANZA_MM:
            if _tempo_in_tolleranza is None:
                _tempo_in_tolleranza = now
            elif now - _tempo_in_tolleranza >= 5.0:
                cv2.putText(img_out, "PASSO 1 COMPLETATO",
                            (20, 140), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 255, 0), 2)
                cam.releaseFrame(frame)
                return True
        else:
            _tempo_in_tolleranza = None

            if now - _last_yaw_time > YAW_RATE_LIMIT_S:
                yaw = min(abs(diff) * (GUADAGNO_YAW / 10.0), MAX_YAW_DEG)
                condition_yaw(vehicle, yaw, -1 if diff < 0 else 1)
                _last_yaw_time = now

    cam.releaseFrame(frame)
    return False
