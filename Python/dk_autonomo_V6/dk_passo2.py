
import time as pytime
from util import *
from config import *


def esegui_passo_2(vehicle, quota_target, depth_buf, img, stato):
    """
    stato:
        0 -> salita fino alla quota target
        1 -> inclinazione videocamera
        2 -> avanti fino a cavo centrato
        3 -> yaw 90 gradi a sinistra
        4 -> fine
    """

    print("[INFO] PASSO 2: salita + ToF down + avanti + yaw SX -->", stato)

    # ---------------- 1. SALITA  ----------------
    if stato == 0:
        if vehicle.location.global_relative_frame.alt < quota_target*0.95:
            set_velocity_body(vehicle, vx=0, vy=0, vz=-VELOCITA_SALITA_MS)
            print(f"[INFO] Salita in corso... Altitudine: {vehicle.location.global_relative_frame.alt:.2f} m")
        else:
            print("[INFO] Quota raggiunta")
            set_velocity_body(vehicle, 0, 0, 0)
            stato = 1

    # ---------------- 2. INCLINAZIONE ToF ----------------
    elif stato == 1:
        set_servo_pwm(vehicle, servo_n=N_SERV_CAM, pwm=PWM_TILT_CAM)
        stato = 2

    # ---------------- 3. AVANZAMENTO FINO A CAVO CENTRATO ----------------
    elif stato == 2:
        # Avanza
        set_velocity_body(vehicle, vx=VELOCITA_AVANTI_PASSO2_MS, vy=0, vz=0)

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

        stato = 4

    return stato


def resta_sopra_il_cavo(vehicle, x_up, x_down, stato_tempo):
     # ---------------- 5. STAZIONE A CAVALLO DEL CAVO ----------------
    print("[INFO] Drone in posizione di sgancio")
    yawok = yaw_control_from_hover(vehicle, x_up, x_down)

    if yawok:
        if pytime.monotonic() - stato_tempo >= TEMPO_STAZIONAMENTO_S:
            return True, stato_tempo
    else:
        stato_tempo = pytime.monotonic()

    return False, stato_tempo
