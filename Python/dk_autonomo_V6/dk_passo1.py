from util import *
from config import *
import time


# ============================================================
# PASSO 1.1 – ALLINEAMENTO PERPENDICOLARE AL CAVO
# ============================================================

def esegui_passo_1(vehicle, d_left, d_right, img, tempo):
    print("[INFO] Esecuzione passo 1")

    passati5sec = False
    posizioneCorretta = yaw_control_from_distances(vehicle, d_left, d_right)
    draw_bottom_center_text(img, "AUTONOMO")


    #verifichiamo che mantenga la posizione per TEMPO_STABILITA_S secondi
    tempo_in_tolleranza = tempo
    if posizioneCorretta:
        if tempo_in_tolleranza is None:
            tempo_in_tolleranza = time.monotonic()
        elif time.monotonic() - tempo_in_tolleranza >= TEMPO_STABILITA_S:
            print("[INFO] Posizione stabile → uscita dal ciclo", time.monotonic() - tempo_in_tolleranza)
            passati5sec = True
    else:
        tempo_in_tolleranza = None

    return tempo_in_tolleranza, passati5sec and posizioneCorretta


# ============================================================
# PASSO 1.2 – MANOVRA DI AVVICINAMENTO E RITIRO
# ============================================================

def manovra_guidata_distanze(vehicle, d_left, d_right, stato, stato_tempo):
    """
    stato:
        0 -> avanzamento fino a DISTANZA_AVVICINAMENTO_MM
        1 -> hover TEMPO_HOVER_S secondi
        2 -> retromarcia fino a DISTANZA_RITIRO_MM
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
    # STATO 0 – AVANTI FINO A DISTANZA_AVVICINAMENTO_MM
    # =====================================================
    if stato == 0:
        if d_media > DISTANZA_AVVICINAMENTO_MM:
            if yaw_ok:
                set_velocity_body(vehicle, vx=VELOCITA_AVANZAMENTO_MS, vy=0, vz=0)
                print(f"[AVANTI] d_media={d_media:.0f} mm")
            else:
                set_velocity_body(vehicle, 0, 0, 0)
        else:
            set_velocity_body(vehicle, 0, 0, 0)
            stato = 1
            stato_tempo = time.monotonic()
            return stato, stato_tempo

    # =====================================================
    # STATO 1 – HOVER
    # =====================================================
    elif stato == 1:
        set_velocity_body(vehicle, 0, 0, 0)
        if time.monotonic() - stato_tempo >= TEMPO_HOVER_S:
            stato = 2
        return stato, stato_tempo

    # =====================================================
    # STATO 2 – INDIETRO FINO A DISTANZA_RITIRO_MM
    # =====================================================
    elif stato == 2:
        if d_media < DISTANZA_RITIRO_MM:
            if yaw_ok:
                set_velocity_body(vehicle, vx=-VELOCITA_AVANZAMENTO_MS, vy=0, vz=0)
                print(f"[INDIETRO] d_media={d_media:.0f} mm")

            else:
                set_velocity_body(vehicle, 0, 0, 0)
        else:
            set_velocity_body(vehicle, 0, 0, 0)
            stato = 3

    return stato, None
