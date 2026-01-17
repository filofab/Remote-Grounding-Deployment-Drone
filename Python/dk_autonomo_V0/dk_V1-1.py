import cv2
import numpy as np
import ArducamDepthCamera as ac
import time

# ============================================================
# PARAMETRI DI FLIP IMMAGINE
# ============================================================

FLIP_HORIZONTAL = True     # mirror sinistra/destra
FLIP_VERTICAL = False     # flip alto/basso


# ============================================================
# PARAMETRI TOF / VISIONE
# ============================================================

MAX_DISTANCE = 4000            # range hardware TOF
MIN_DISTANCE_MM = 200
MAX_DISTANCE_MM = 2000

CONFIDENCE_THRESHOLD = 30

# Linee verticali simmetriche rispetto al centro
VERTICAL_LINE_SPACING_PX = 200
ROI_WIDTH_PX = 20              # larghezza reale ROI verticale
PERCENTILE_DISTANCE = 10       # percentile robusto (outlier rejection)


# ============================================================
# PARAMETRI DRONEKIT (solo scaffold)
# ============================================================

ENABLE_DRONE = True           # per ora OFF
CONNECTION_STRING = "udpin:0.0.0.0:14550"
TARGET_ALTITUDE_M = 2.0


# ============================================================
# FUNZIONI DI SUPPORTO
# ============================================================

def apply_flip(img):
    """
    Applica flip orizzontale e/o verticale in base ai parametri globali.
    """
    if FLIP_HORIZONTAL and FLIP_VERTICAL:
        return cv2.flip(img, -1)
    elif FLIP_HORIZONTAL:
        return cv2.flip(img, 1)
    elif FLIP_VERTICAL:
        return cv2.flip(img, 0)
    return img


# ============================================================
# FUNZIONI DRONEKIT (BASE, NON USATE ANCORA)
# ============================================================

if ENABLE_DRONE:
    from dronekit import connect, VehicleMode
    from pymavlink import mavutil


def arm_and_takeoff(vehicle, target_altitude):
    print("[INFO] Pre-arm checks")
    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        time.sleep(0.5)

    vehicle.simple_takeoff(target_altitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        if alt >= target_altitude * 0.95:
            break
        time.sleep(0.5)


# ============================================================
# MISURA ROBUSTA LUNGO LINEA VERTICALE
# ============================================================

def measure_distance_vertical(depth, confidence, x_center):
    """
    Estrae una distanza robusta lungo una ROI verticale centrata in x_center.
    Usa percentile per ridurre rumore TOF e spike.
    """

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

    # Troppi pochi punti -> misura non affidabile
    if valid_depths.size < 50:
        return None, None, None

    # Stima robusta della distanza
    dist = np.percentile(valid_depths, PERCENTILE_DISTANCE)

    # Coordinate rappresentative dei punti più vicini
    mask_close = valid & (roi_depth <= dist)
    ys, xs = np.where(mask_close)

    if ys.size == 0:
        return None, None, None

    x_mean = int(xs.mean() + x_min)
    y_mean = int(ys.mean())

    return int(dist), x_mean, y_mean


# ============================================================
# MAIN
# ============================================================

def main():

    print("Arducam TOF Preview + DroneKit scaffold")
    print("SDK version:", ac.__version__)

    # ---------- CAMERA ----------
    cam = ac.ArducamCamera()
    ret = cam.open(ac.Connection.CSI, 0)
    if ret != 0:
        print("Errore apertura camera")
        return

    cam.start(ac.FrameType.DEPTH)
    cam.setControl(ac.Control.RANGE, MAX_DISTANCE)

    info = cam.getCameraInfo()
    print(f"Risoluzione: {info.width}x{info.height}")

    # ---------- DRONE (non attivo) ----------
    if ENABLE_DRONE:
        vehicle = connect(CONNECTION_STRING, wait_ready=True, timeout=60)
        arm_and_takeoff(vehicle, TARGET_ALTITUDE_M)

    cv2.namedWindow("preview", cv2.WINDOW_AUTOSIZE)

    while True:
        frame = cam.requestFrame(2000)

        if frame is not None and isinstance(frame, ac.DepthData):

            # Flip applicato SUBITO (coerenza geometrica)
            depth = apply_flip(frame.depth_data)
            confidence = apply_flip(frame.confidence_data)

            # Visualizzazione depth
            img = (depth * (255.0 / MAX_DISTANCE)).astype(np.uint8)
            img = cv2.applyColorMap(img, cv2.COLORMAP_RAINBOW)
            img[confidence < CONFIDENCE_THRESHOLD] = (0, 0, 0)

            h, w = depth.shape
            cx = w // 2

            # Linee verticali SX / DX
            x_left = cx - VERTICAL_LINE_SPACING_PX // 2
            x_right = cx + VERTICAL_LINE_SPACING_PX // 2

            cv2.line(img, (x_left, 0), (x_left, h), (255, 255, 255), 1)
            cv2.line(img, (x_right, 0), (x_right, h), (255, 255, 255), 1)

            # Misure robuste
            d_left, xl, yl = measure_distance_vertical(depth, confidence, x_left)
            d_right, xr, yr = measure_distance_vertical(depth, confidence, x_right)

            # Overlay risultati
            if d_left is not None:
                cv2.circle(img, (xl, yl), 6, (0, 0, 255), -1)
                cv2.putText(
                    img, f"L: {d_left} mm",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 0, 255), 2
                )

            if d_right is not None:
                cv2.circle(img, (xr, yr), 6, (0, 0, 255), -1)
                cv2.putText(
                    img, f"R: {d_right} mm",
                    (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 0, 255), 2
                )

            cv2.imshow("preview", img)
            cam.releaseFrame(frame)

        key = cv2.waitKey(1)
        if key == ord("q"):
            break

    # ---------- CLEANUP ----------
    if ENABLE_DRONE:
        vehicle.close()

    cam.stop()
    cam.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
