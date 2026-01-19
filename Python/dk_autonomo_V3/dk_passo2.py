
import time as pytime
from util import *


def esegui_passo_2(vehicle, quota_target, depth_buf, img, stato):
    """
    stato:
        0 -> salita 1m
        1 -> inclinazione videocamera
        2 -> avanti 1m
        3 -> yaw 90 gradi a sinistra
        4 -> fine
    """

    print("[INFO] PASSO 2: salita + ToF down + avanti + yaw SX -->", stato)

    # ---------------- 1. SALITA +1 m ----------------
    if stato == 0:
        if vehicle.location.global_relative_frame.alt < quota_target*0.95:
            set_velocity_body(vehicle, vx=0, vy=0, vz=-0.15)
            print(f"[INFO] Salita in corso... Altitudine: {vehicle.location.global_relative_frame.alt:.2f} m")
            pytime.sleep(0.2)

        if vehicle.location.global_relative_frame.alt >= quota_target*0.95:
            print("[INFO] Quota raggiunta")
            stato = 1

    # ---------------- 2. INCLINAZIONE ToF (AUX9) ----------------
    elif stato == 1:
        PWM_90_DOWN = 1900
        set_servo_pwm(vehicle, servo_n=9, pwm=PWM_90_DOWN)
        stato = 2

    # ---------------- 3. AVANZAMENTO 1 m ----------------
    elif stato == 2:
        VELOCITA_AVANTI = 0.2

        # Avanza
        set_velocity_body(vehicle, vx=VELOCITA_AVANTI, vy=0, vz=0)

        centrato, y_cavo = cavo_al_centro(depth_buf)

        if y_cavo is not None:
            cv2.line(img, (0, y_cavo), (img.shape[1], y_cavo), (0, 255, 0), 2)

        if centrato:
            print("[INFO] Cavo centrato verticalmente")

            # STOP immediato
            set_velocity_body(vehicle, 0, 0, 0)
            stato = 3
        

    # ---------------- 4. YAW 90° A SINISTRA ----------------
    elif stato == 3:
        print("[INFO] Rotazione YAW 90° sinistra")
        condition_yaw(vehicle, 90, direction=-1)

        print("[INFO] PASSO 2 completato")
        stato = 4

    return stato