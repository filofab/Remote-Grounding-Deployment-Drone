
import time as pytime
from util import *


def esegui_passo_2(vehicle):
    print("[INFO] PASSO 2: salita + ToF down + avanti + yaw SX")

    # ---------------- 1. SALITA +1 m ----------------
    quota_attuale = vehicle.location.global_relative_frame.alt
    quota_target = quota_attuale + 1.0

    vehicle.simple_takeoff(quota_target)
    while vehicle.location.global_relative_frame.alt < quota_target * 0.95:
        pytime.sleep(0.2)

    print("[INFO] Quota raggiunta")

    # ---------------- 2. INCLINAZIONE ToF (AUX9) ----------------
    PWM_90_DOWN = 1900
    set_servo_pwm(vehicle, servo_n=9, pwm=PWM_90_DOWN)
    pytime.sleep(1)

    # ---------------- 3. AVANZAMENTO 1 m ----------------
    VELOCITA_AVANTI = 0.2      # m/s
    DISTANZA = 1.0             # m
    DURATA = DISTANZA / VELOCITA_AVANTI

    send_velocity_body(
        vehicle,
        vx=VELOCITA_AVANTI,
        vy=0,
        vz=0,
        duration=DURATA
    )

    print("[INFO] Avanzamento completato")

    # ---------------- 4. YAW 90° A SINISTRA ----------------
    print("[INFO] Rotazione YAW 90° sinistra")
    condition_yaw(vehicle, 90, direction=-1)

    pytime.sleep(3)  # tempo per completare la rotazione

    print("[INFO] PASSO 2 completato")
    return True