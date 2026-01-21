
import ArducamDepthCamera as ac
import time as pytime

from util import *
from dk_passo1 import esegui_passo_1, manovra_guidata_distanze
from dronekit import connect, VehicleMode
from dk_passo2 import esegui_passo_2, resta_sopra_il_cavo
from config import *



# ============================================================
# FUNZIONI DRONEKIT
# ============================================================

def arm_and_takeoff(vehicle, target_altitude):
    print("[INFO] Pre-arm checks...")
    while not vehicle.is_armable:
        pytime.sleep(1)

    print("[INFO] Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        pytime.sleep(0.5)

    print(f"[INFO] Taking off to {target_altitude} m")
    vehicle.simple_takeoff(target_altitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        if alt >= target_altitude * 0.95:
            print("[OK] Target altitude reached")
            break
        pytime.sleep(0.5)



def main():


    print("[INFO] Avvio sistema TOF + DroneKit")

    # ---------- DRONE ----------
    print("[INFO] Connessione al drone...")
    vehicle = connect(CONNECTION_STRING, wait_ready=True, timeout=120)
    arm_and_takeoff(vehicle, TARGET_ALTITUDE_M)

    # ---------- CAMERA ----------
    cam = ac.ArducamCamera()
    cam.open(ac.Connection.CSI, 0)
    cam.start(ac.FrameType.DEPTH)
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
    mostra_popup = False  # variabile per la gestione dei popup
    tempo=None  #variabile per il controllo del tempo in tolleranza
    stato1_2=0  #variabile per la gestione
    stato_tempo=None
    stato2 = 0
    attesa_conferma1_1 = False
    attesa_conferma1_2 = False
    attesa_conferma2 = False
    attesa_conferma3 = False
    cima_sganciata = False
    persona_detected = False
    altezza_corrente = vehicle.location.global_relative_frame.alt
    d_left, d_right, x_up, x_down = None, None, None, None
    while True:
        frame = cam.requestFrame(2000)

        if frame is not None and isinstance(frame, ac.DepthData):

            depth = apply_flip(frame.depth_data)
            confidence = apply_flip(frame.confidence_data)

            img = (depth * (255.0 / MAX_DISTANCE)).astype(np.uint8)
            img = cv2.applyColorMap(img, cv2.COLORMAP_RAINBOW)
            img[confidence < CONFIDENCE_THRESHOLD] = (0, 0, 0)

            h, w = depth.shape

            if stato2 != 4:

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
            else:

                cy = h // 2
                y_down = cy - VERTICAL_LINE_SPACING_PX // 2
                y_up = cy + VERTICAL_LINE_SPACING_PX // 2

                cv2.line(img, (0, y_down), (w, y_down), (255, 255, 255), 1)
                cv2.line(img, (0, y_up), (w, y_up), (255, 255, 255), 1)

                d_down, xl, yl = measure_distance_horizontal(depth, confidence, y_down)
                d_up, xr, yr = measure_distance_horizontal(depth, confidence, y_up)

                if d_down is not None:

                    cv2.circle(img, (xl, yl), 6, (0, 0, 255), -1)
                    cv2.putText(img, f"L: {d_down} mm", (20, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    x_down = xl

                if d_up is not None:

                    cv2.circle(img, (xr, yr), 6, (0, 0, 255), -1)
                    cv2.putText(img, f"R: {d_up} mm", (20, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                    x_up = xr


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
                if stato1_2 == 3:
                    attesa_conferma1_1 = False
                    attesa_conferma1_2 = True
                    altezza_corrente = vehicle.location.global_relative_frame.alt
                    stato_tempo = None

            elif token == 3: #passo 2
                stato2 = esegui_passo_2(vehicle, altezza_corrente+INCREMENTO_ALTEZZA, frame.depth_data, img, stato2)
                if stato2 == 4:
                    attesa_conferma1_2 = False
                    token = 4
                    stato_tempo = pytime.monotonic()
                    attesa_conferma3 = True

            elif token == 4:
                attesa_conferma3, stato_tempo = resta_sopra_il_cavo(vehicle, x_up, x_down, stato_tempo)




            ######################### COMUNICAZIONE STATI PREVIEW ###################################
            if attesa_conferma1_1 or attesa_conferma1_2 or attesa_conferma2 or attesa_conferma3 or cima_sganciata or persona_detected:

                if attesa_conferma1_1:
                    title = "Allineamento completato"
                elif attesa_conferma1_2:
                    title = "Verifica completata"
                elif attesa_conferma2:
                    title = "Posizionamento azimutale completato"
                elif attesa_conferma3:
                    title = "PRONTO A SGANCIARE (premi d)"
                elif cima_sganciata:
                    title = "GUIDA SGANCIATA (r -> guida controllata)"
                elif persona_detected:
                    title = "PERSONA RILEVATA!"

                lines = [
                    (title, (255, 255, 255), FONT_SCALE_TITLE),
                    ("y = continua", (0, 255, 0), FONT_SCALE_CMD),
                    ("r = reset", (0, 0, 255), FONT_SCALE_CMD),
                ]





                h, w = img.shape[:2]
                sizes = [cv2.getTextSize(t, FONT, s, FONT_THICK)[0] for t, _, s in lines]

                box_w = max(sw for sw, sh in sizes) + 2 * PADDING
                box_h = sum(sh for sw, sh in sizes) + (len(lines) + 1) * PADDING

                x0 = (w - box_w) // 2
                y0 = (h - box_h) // 2

                # sfondo nero
                cv2.rectangle(
                    img,
                    (x0, y0),
                    (x0 + box_w, y0 + box_h),
                    (0, 0, 0),
                    -1
                )

                # testo
                y = y0 + PADDING + sizes[0][1]
                for (text, color, scale), (tw, th) in zip(lines, sizes):
                    cv2.putText(
                        img,
                        text,
                        (x0 + (box_w - tw) // 2, y),
                        FONT,
                        scale,
                        color,
                        FONT_THICK,
                        cv2.LINE_AA
                    )

                    y += th + PADDING

            cv2.imshow("preview", img)
            cam.releaseFrame(frame)





            ######################### COMUNICAZIONE TASTIERA ###################################
        key = cv2.waitKey(1) & 0xFF

        if key == ord("y"):
            print("[MODE] Missione autonoma ATTIVA")
            token += 1

            mostra_popup = False
            attesa_conferma1_1 = False
            attesa_conferma1_2 = False
            attesa_conferma2 = False

        elif key == ord("r"):
            print("[INFO] Reset Passo 0")

            stato1_2 = 0
            stato_tempo=None
            stato2 = 0

            mostra_popup = False
            tempo = None
            token = 0

            attesa_conferma1_1 = False
            attesa_conferma1_2 = False
            attesa_conferma2 = False

        elif key == ord("s"):
            print("[MODE] Missione in PAUSA")
            token = 0

        elif key == ord("l"):
            print("[MODE] Atterraggio")
            token = 0
            vehicle.mode = VehicleMode("LAND")

        elif key == ord("q"):
            print("[EXIT] Chiusura sistema")
            break

        elif key == ord("d"):
            print("[ACTION] Sgancio carico")
            stato_tempo = pytime.monotonic()
            attesa_conferma3 = False
            cima_sganciata = True

        elif key == ord("p"):
            if persona_detected:
                print("[OK] Nessuna persona rilevata!")
                persona_detected = False
            else:
                print("[WARNING] Persona rilevata!")
                attesa_conferma3 = False
                cima_sganciata = False
                persona_detected = True


    # ---------- CLEANUP ----------
    vehicle.close()
    cam.stop()
    cam.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()