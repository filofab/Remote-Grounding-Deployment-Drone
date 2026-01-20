from datetime import time
from pymavlink import mavutil
from config import *
import cv2
import numpy as np

# ---------------- ELEMENTI VISIVI ----------------
def apply_flip(img):
    if FLIP_HORIZONTAL and FLIP_VERTICAL:
        return cv2.flip(img, -1)
    elif FLIP_HORIZONTAL:
        return cv2.flip(img, 1)
    elif FLIP_VERTICAL:
        return cv2.flip(img, 0)
    return img


def draw_center_text(img, text,
                     font=cv2.FONT_HERSHEY_SIMPLEX,
                     scale=1.2,
                     thickness=2,
                     padding=10):

    h_img, w_img = img.shape[:2]
    (w_txt, h_txt), baseline = cv2.getTextSize(text, font, scale, thickness)

    # Coordinate testo (centrato)
    x = (w_img - w_txt) // 2
    y = (h_img + h_txt) // 2

    # Rettangolo sfondo nero
    cv2.rectangle(
        img,
        (x - padding, y - h_txt - padding),
        (x + w_txt + padding, y + baseline + padding),
        (0, 0, 0),
        -1
    )

    # Testo bianco
    cv2.putText(
        img,
        text,
        (x, y),
        font,
        scale,
        (255, 255, 255),
        thickness,
        cv2.LINE_AA
    )

def draw_bottom_center_text(img, text,
                            font=cv2.FONT_HERSHEY_SIMPLEX,
                            scale=0.9,
                            thickness=2,
                            margin_bottom=20,
                            padding=10):

    h_img, w_img = img.shape[:2]
    (w_txt, h_txt), baseline = cv2.getTextSize(text, font, scale, thickness)

    # Coordinate testo (centrato in basso)
    x = (w_img - w_txt) // 2
    y = h_img - margin_bottom

    # Rettangolo sfondo nero
    cv2.rectangle(
        img,
        (x - padding, y - h_txt - padding),
        (x + w_txt + padding, y + baseline + padding),
        (0, 0, 0),
        -1
    )

    # Testo bianco
    cv2.putText(
        img,
        text,
        (x, y),
        font,
        scale,
        (255, 255, 255),
        thickness,
        cv2.LINE_AA
    )



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



def measure_distance_horizontal(depth, confidence, y_center):

    h, w = depth.shape

    y_min = max(0, y_center - ROI_WIDTH_PX // 2)
    y_max = min(h, y_center + ROI_WIDTH_PX // 2)

    roi_depth = depth[y_min:y_max, :]
    roi_conf  = confidence[y_min:y_max, :]

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

    x_mean = int(xs.mean())
    y_mean = int(ys.mean() + y_min)

    return int(dist), x_mean, y_mean


def condition_yaw(vehicle, heading, direction=1):
    """
    direction:
        1  -> CW  (destra)
       -1  -> CCW (sinistra)
    """
    dir_val = 1 if direction == 1 else -1
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


# ---------------- SERVO AUX ----------------

def set_servo_pwm(vehicle, servo_n, pwm):
    """
    servo_n: numero servo (9 = AUX9)
    pwm: valore PWM (1000–2000)
    """
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        servo_n,
        pwm,
        0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# ---------------- MOVIMENTO VELOCITÀ ----------------
#ATTENZIONE FUNZIONA CON IL TEMPO
def send_velocity_body(vehicle, vx, vy, vz, duration):
    """
    vx: avanti (m/s)
    vy: destra (m/s)
    vz: giù (m/s) -> negativo per salire
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )

    for _ in range(int(duration * 10)):
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

# invio velocità senza durata
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

# funzioni per vedere se il cavo è centrale
def trova_y_cavo(depth_buf):
    """
    Restituisce la coordinata Y del cavo oppure None
    """
    h, w = depth_buf.shape

    # Filtri base
    valid = (depth_buf > 200) & (depth_buf < 2000)

    # distanza minima per ogni riga
    min_per_row = np.where(valid, depth_buf, np.inf).min(axis=1)

    # riga con oggetto più vicino
    y_cavo = np.argmin(min_per_row)

    # verifica che sia DAVVERO un cavo (non rumore)
    if min_per_row[y_cavo] < 800:  # mm → da tarare
        return y_cavo

    return None

def cavo_al_centro(depth_buf, tolleranza_px=20):
    h, _ = depth_buf.shape
    y_cavo = trova_y_cavo(depth_buf)

    if y_cavo is None:
        return False, None

    centro = h // 2

    if abs(y_cavo - centro) < tolleranza_px:
        return True, y_cavo

    return False, y_cavo