

import cv2
import numpy as np
import ArducamDepthCamera as ac
import time

from util import *
from dk_passo1 import esegui_passo_1, manovra_guidata_distanze
from dronekit import connect, VehicleMode
from config import (
    SOGLIA_TOLLERANZA_MM,
    GUADAGNO_YAW,
    MAX_YAW_DEG,
    ROI_WIDTH_PX,
    MIN_DISTANCE_MM,
    PERCENTILE_DISTANCE,
    CONFIDENCE_THRESHOLD,
    MAX_DISTANCE_MM,
    VERTICAL_LINE_SPACING_PX,
    MAX_DISTANCE,
    CONNECTION_STRING,
    TARGET_ALTITUDE_M
)
import config



# ============================================================
# FUNZIONI DRONEKIT
# ============================================================

def arm_and_takeoff(vehicle, target_altitude):
    print("[INFO] Pre-arm checks...")
    while not vehicle.is_armable:
        time.sleep(1)

    print("[INFO] Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        time.sleep(0.5)

    print(f"[INFO] Taking off to {target_altitude} m")
    vehicle.simple_takeoff(target_altitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        if alt >= target_altitude * 0.95:
            print("[OK] Target altitude reached")
            break
        time.sleep(0.5)






def main():

    print("[INFO] Avvio sistema TOF + DroneKit")

    # ---------- DRONE ----------
    print("[INFO] Connessione al drone...")
    vehicle = connect(CONNECTION_STRING, wait_ready=True, timeout=60)
    arm_and_takeoff(vehicle, TARGET_ALTITUDE_M)

    # ---------- CAMERA ----------
    cam = ac.ArducamCamera()
    cam.open(ac.Connection.CSI, 0)
    cam.start(ac.FrameType.DEPTH | ac.FrameType.RGB)
    cam.setControl(ac.Control.RANGE, MAX_DISTANCE)

    info = cam.getCameraInfo()
    print(f"[INFO] Risoluzione camera: {info.width}x{info.height}")

    cv2.namedWindow("preview", cv2.WINDOW_AUTOSIZE)



    print("\n--- COMANDI ---")
    print("y : avvia missione autonoma")
    print("s : pausa missione")
    print("l : atterraggio")
    print("q : chiusura")


    token=0 #-> token che identifica il passo, 0 guida controllata
    tempo=None  #variabile per il controllo del tempo in tolleranza
    stato1_2=0  #variabile per la gestione
    stato_tempo=None
    attesa_conferma1_1 = False
    while True:
        frame = cam.requestFrame(2000)

        if frame is None:
            continue
        if isinstance(frame, ac.DepthData):

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
        elif isinstance(frame, ac.RGBData) and token == 4:
            rgb_frame = apply_flip(frame.rgb_data)

            ############################################################
            """
            se il token è 0 -> guida controllata
            token 1 -> guida autonoma passo 1
            token 2 -> guida autonoma passo 2
            token 3 guida autonoma passo 3
            """
            if token == 1: #passo 1.1
                tempo, finito = esegui_passo_1(vehicle, d_left, d_right, img, tempo)
                if finito:
                    attesa_conferma1_1 = True

            elif token == 2: #passo 1.2
                stato1_2, stato_tempo = manovra_guidata_distanze( vehicle, d_left, d_right, stato1_2, stato_tempo)

            elif token == 3: #passo 2
                # esegui_passo_2()
                pass

            elif token == 4: #passo 3 (AI)
                # esegui_passo_3()
                pass

            ############################################################
            if attesa_conferma1_1:
                cv2.putText(img, "Passo 1 completato",
                            (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (0, 255, 0), 2)

                cv2.putText(img, "Y = continua | R = reset",
                            (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 255), 2)

            cv2.imshow("preview", img)
            cam.releaseFrame(frame)




        key = cv2.waitKey(1) & 0xFF
        if key == ord("y"):
            token+=1
            print("[MODE] Missione autonoma ATTIVA")

        elif key == ord("s"):  #TODO
            mission_active = False
            print("[MODE] Missione in PAUSA")

        elif key == ord("l"):
            print("[MODE] Atterraggio")
            vehicle.mode = VehicleMode("LAND")

        elif key == ord('r'):
            print("[INFO] Reset Passo 0")
            tempo = None
            attesa_conferma1_1 = False
            token = 0

        elif key == ord("q"):
            print("[EXIT] Chiusura sistema")
            break

    # ---------- CLEANUP ----------
    vehicle.close()
    cam.stop()
    cam.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()