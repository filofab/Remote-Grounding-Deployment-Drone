from pymavlink import mavutil
from config import FLIP_HORIZONTAL, FLIP_VERTICAL,ROI_WIDTH_PX,MIN_DISTANCE_MM,MAX_DISTANCE_MM,CONFIDENCE_THRESHOLD,PERCENTILE_DISTANCE
import cv2
import numpy as np


def apply_flip(img):
    if FLIP_HORIZONTAL and FLIP_VERTICAL:
        return cv2.flip(img, -1)
    elif FLIP_HORIZONTAL:
        return cv2.flip(img, 1)
    elif FLIP_VERTICAL:
        return cv2.flip(img, 0)
    return img


def measure_distance_vertical(depth, confidence, x_center):

    h, w = depth.shape

    x_min = max(0, x_center - ROI_WIDTH_PX // 2)
    x_max = min(w, x_center + ROI_WIDTH_PX // 2)

    roi_depth = depth[:, x_min:x_max]
    roi_conf = confidence[:, x_min:x_max]

    valid = (
        (roi_depth >= MIN_DISTANCE_MM) &
        (roi_depth <= MAX_DISTANCE_MM) &
        (roi_conf >= CONFIDENCE_THRESHOLD)
    )

    valid_depths = roi_depth[valid]

    if valid_depths.size < 50:
        return None, None, None

    dist = np.percentile(valid_depths, PERCENTILE_DISTANCE)

    mask_close = valid & (roi_depth <= dist)
    ys, xs = np.where(mask_close)

    if ys.size == 0:
        return None, None, None

    x_mean = int(xs.mean() + x_min)
    y_mean = int(ys.mean())

    return int(dist), x_mean, y_mean


def set_velocity_body(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)
