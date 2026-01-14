

import cv2
import numpy as np
import ArducamDepthCamera as ac
import time

from dk_passo1 import esegui_passo_1
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
    cam.start(ac.FrameType.DEPTH)
    cam.setControl(ac.Control.RANGE, MAX_DISTANCE)

    info = cam.getCameraInfo()
    print(f"[INFO] Risoluzione camera: {info.width}x{info.height}")

    cv2.namedWindow("preview", cv2.WINDOW_AUTOSIZE)

    mission_active = False

    print("\n--- COMANDI ---")
    print("y : avvia missione autonoma")
    print("s : pausa missione")
    print("l : atterraggio")
    print("q : chiusura")
    token=0 #-> token che identifica il passo, 0 guida controllata
    while True:

        """
        se il token è 0 -> guida controllata
        token 1 -> guida autonoma passo 1
        token 2 -> guida autonoma passo 2
        token 3 guida autonoma passo 3
        """
        if token==1:
            esegui_passo_1(vehicle, cam)
        elif token==2:
            #esegui_passo_2()
            pass
        elif token==3:
            #esegui_passo_3()
            pass




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