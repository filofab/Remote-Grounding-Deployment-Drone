import cv2
import ArducamDepthCamera as ac
from pymavlink import mavutil
from config import *

import numpy as np
from util import *
from config import *
import time

tempo = None








# ============================================================
# VISIONE TOF – MISURA ROBUSTA
# ============================================================




def esegui_passo_1(vehicle, d_left, d_right, img, tempo):
    print("[INFO] Esecuzione passo 1")

    passati5sec=False
    posizioneCorretta = yaw_control_from_distances(vehicle, d_left, d_right)
    draw_bottom_center_text(img, "AUTONOMO")


    #verifichiamo che mantenga la posizione per 5 secondi
    tempo_in_tolleranza = tempo
    if posizioneCorretta:
        if tempo_in_tolleranza is None:
            tempo_in_tolleranza = time.monotonic()
        elif time.monotonic() - tempo_in_tolleranza >= 5.0:
            print("[INFO] Posizione stabile per 5 secondi → uscita dal ciclo", time.monotonic() - tempo_in_tolleranza)
            passati5sec=True
    else:
        tempo_in_tolleranza = None
    
    return tempo_in_tolleranza, passati5sec and posizioneCorretta

    #procediamo ad andare avanti di 1m e indietro di 1m

def manovra_guidata_distanze(vehicle, d_left, d_right, stato, stato_tempo):
    """
    stato:
        0 -> avanzamento fino a 40 cm
        1 -> hover 3 s
        2 -> retromarcia fino a 1 m
        3 -> fine
    """
    print("-----------------------------------------------------", stato)

    if d_left is None or d_right is None:
        set_velocity_body(vehicle, 0, 0, 0)
        return stato, stato_tempo

    d_media = (d_left + d_right) / 2.0

    # --- ALLINEAMENTO SEMPRE ATTIVO ---
    yaw_ok = yaw_control_from_distances(vehicle, d_left, d_right)

    # =====================================================
    # STATO 0 – AVANTI FINO A 40 cm
    # =====================================================
    if stato == 0:
        if d_media > 400:
            if yaw_ok:
                set_velocity_body(vehicle, vx=0.15, vy=0, vz=0)
                print(f"[AVANTI] d_media={d_media:.0f} mm")
            else:
                set_velocity_body(vehicle, 0, 0, 0)
        else:
            set_velocity_body(vehicle, 0, 0, 0)
            stato = 1
            stato_tempo = time.monotonic()
            return stato, stato_tempo

    # =====================================================
    # STATO 1 – HOVER 3 s
    # =====================================================
    elif stato == 1:
        set_velocity_body(vehicle, 0, 0, 0)
        if time.monotonic() - stato_tempo >= 3.0:
            stato = 2
        return stato, stato_tempo

    # =====================================================
    # STATO 2 – INDIETRO FINO A 1 m
    # =====================================================
    elif stato == 2:
        if d_media < 1000:
            if yaw_ok:
                set_velocity_body(vehicle, vx=-0.15, vy=0, vz=0)
                print(f"[INDIETRO] d_media={d_media:.0f} mm")

            else:
                set_velocity_body(vehicle, 0, 0, 0)
        else:
            set_velocity_body(vehicle, 0, 0, 0)
            stato = 3

    return stato, None

