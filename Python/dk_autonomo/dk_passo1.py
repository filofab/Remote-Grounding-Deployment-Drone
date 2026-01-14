import cv2
import ArducamDepthCamera as ac
from pymavlink import mavutil
from main import SOGLIA_TOLLERANZA_MM, GUADAGNO_YAW, MAX_YAW_DEG, ROI_WIDTH_PX, MIN_DISTANCE_MM, PERCENTILE_DISTANCE, CONFIDENCE_THRESHOLD, MAX_DISTANCE_MM
import numpy as np
from main import apply_flip, MAX_DISTANCE, VERTICAL_LINE_SPACING_PX


def condition_yaw(vehicle, heading, direction=1):
    """
    direction:
        1  -> CW  (destra)
       -1  -> CCW (sinistra)
    """
    dir_val = 1 if direction == 1 else -1
    print(dir_val)
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


def yaw_control_from_distances(vehicle, d_left, d_right):
    """
    Controllo yaw proporzionale basato sulla differenza
    tra distanza sinistra e destra.
    """

    if d_left is None or d_right is None:
        return False

    diff = d_left - d_right

    if abs(diff) <= SOGLIA_TOLLERANZA_MM:
        return True

    yaw_cmd = min(abs(diff) * (GUADAGNO_YAW / 10.0), MAX_YAW_DEG)

    if diff < 0:
        condition_yaw(vehicle, yaw_cmd, direction=-1)
        print(f"[CTRL] YAW SINISTRA | diff {diff} mm")
    else:
        condition_yaw(vehicle, yaw_cmd, direction=1)
        print(f"[CTRL] YAW DESTRA  | diff {diff} mm")


# ============================================================
# VISIONE TOF – MISURA ROBUSTA
# ============================================================

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




def esegui_passo_1(vehicle, cam):

    while True and possoProcedere:
        frame = cam.requestFrame(2000)

        if frame is not None and isinstance(frame, ac.DepthData):

            depth = apply_flip(frame.depth_data)
            confidence = apply_flip(frame.confidence_data)

            img = (depth * (255.0 / MAX_DISTANCE)).astype(np.uint8)
            img = cv2.applyColorMap(img, cv2.COLORMAP_RAINBOW)
            img[confidence < CONFIDENCE_THRESHOLD] = (0, 0, 0)

            h, w = depth.shape
            cx = w // 2
            x_left = cx - VERTICAL_LINE_SPACING_PX // 2
            x_right = cx + VERTICAL_LINE_SPACING_PX // 2

            cv2.line(img, (x_left, 0), (x_left, h), (255, 255, 255), 1)
            cv2.line(img, (x_right, 0), (x_right, h), (255, 255, 255), 1)

            d_left, xl, yl = measure_distance_vertical(depth, confidence, x_left)
            d_right, xr, yr = measure_distance_vertical(depth, confidence, x_right)

            if d_left is not None:
                cv2.circle(img, (xl, yl), 6, (0, 0, 255), -1)
                cv2.putText(img, f"L: {d_left} mm", (20, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            if d_right is not None:
                cv2.circle(img, (xr, yr), 6, (0, 0, 255), -1)
                cv2.putText(img, f"R: {d_right} mm", (20, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            
            posizioneCoretta = yaw_control_from_distances(vehicle, d_left, d_right)
            cv2.putText(img, "AUTONOMO", (20, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            if posizioneCorretta:
                passati5sec=True
                time.
                while posizioneCorretta and passati5sec:
                    posizioneCoretta = yaw_control_from_distances(vehicle, d_left, d_right)

            cv2.imshow("preview", img)
            cam.releaseFrame(frame)

        
