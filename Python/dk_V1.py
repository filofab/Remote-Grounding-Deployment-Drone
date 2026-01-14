
import cv2
import numpy as np
import ArducamDepthCamera as ac
import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# ---------------- CONFIG CAMERA ----------------
MAX_DISTANCE = 1000  
GRID_COLS = 3  
MIN_DISTANCE = 200  
MAX_DISTANCE_FILTER = 4000  
confidence_value = 1

# ---------------- CONFIG DRONE ----------------
CONNECTION_STRING = 'udpin:0.0.0.0:14550'
TARGET_ALTITUDE = 1.0       
GUADAGNO_YAW = 0.5          # Sensibilità rotazione
SOGLIA_TOLLERANZA_MM = 30   # Tolleranza tra i due punti (3cm)


# ---------------- FUNZIONI VOLO ----------------

def arm_and_takeoff(vehicle, aTargetAltitude):
    print("Pre-arm checks...")
    while not vehicle.is_armable:
        print(" In attesa del GPS/Sensori...")
        time.sleep(1)

    print("Armando e passando in GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        time.sleep(1)

    print(f"Decollo... Target: {aTargetAltitude}m")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Altitudine: {alt:.1f}m")
        if alt >= aTargetAltitude * 0.95:
            print("Quota raggiunta!")
            break
        time.sleep(1)

def condition_yaw(vehicle, heading, direction=1):
    """ direction: 1=CW (Destra), -1=CCW (Sinistra) """
    dir_val = 1 if direction == 1 else 0
    print(dir_val)
    msg = vehicle.message_factory.command_long_encode(
        0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
        heading, 0, dir_val, 1, 0, 0, 0) 
    vehicle.send_mavlink(msg)

def esegui_correzione_yaw(vehicle, points):
    if points[0] is None or points[1] is None:
        return 

    dist_sx = points[0]['dist']
    dist_dx = points[1]['dist']
    diff = dist_sx - dist_dx  

    if abs(diff) > SOGLIA_TOLLERANZA_MM:
        # Più la differenza è alta, più gradi di rotazione chiediamo
        angolo = min(abs(diff) * (GUADAGNO_YAW / 10), 15) 
        
        if diff < 0: # Sinistra è più vicino
            condition_yaw(vehicle, heading=angolo, direction=-1)
            print(f"[*] Correzione SINISTRA: diff {abs(diff)/10:.1f}cm")
        else: # Destra è più vicino
            condition_yaw(vehicle, heading=angolo, direction=1)
            print(f"[*] Correzione DESTRA: diff {abs(diff)/10:.1f}cm")

# ---------------- FUNZIONI VISIONE ----------------

def find_closest_points(depth_buf, conf_buf):
    h, w = depth_buf.shape
    col_w = w // GRID_COLS
    valid = (depth_buf >= MIN_DISTANCE) & (depth_buf <= MAX_DISTANCE_FILTER)
    if conf_buf is not None:
        valid &= (conf_buf >= confidence_value)
    
    punti = []
    for col in [0, GRID_COLS-1]: # Solo colonna 0 (SX) e ultima (DX)
        m = np.zeros_like(depth_buf, dtype=bool)
        m[:, col*col_w : (col+1)*col_w] = True
        m &= valid
        if np.any(m):
            d_masked = np.where(m, depth_buf, np.inf)
            idx = np.argmin(d_masked)
            y, x = np.unravel_index(idx, depth_buf.shape)
            punti.append({'x': x, 'y': y, 'dist': depth_buf[y, x]})
        else:
            punti.append(None)
    return punti

# ---------------- MAIN ----------------

def main():
    print(f"Connessione al drone...")
    try:
        vehicle = connect(CONNECTION_STRING, wait_ready=True, timeout=60)
        arm_and_takeoff(vehicle, TARGET_ALTITUDE)
    except Exception as e:
        print(f"Errore: {e}")
        return

    cam = ac.ArducamCamera()
    if cam.open(ac.Connection.CSI, 0) != 0: return
    cam.start(ac.FrameType.DEPTH)
    cam.setControl(ac.Control.RANGE, MAX_DISTANCE)
    
    cv2.namedWindow("Controllo Drone")
    correzione_attiva = False

    print("\n--- PRONTO ALL'USO ---")
    print("Premi 'y' per ATTIVARE la correzione")
    print("Premi 's' per STOPPARE la correzione")
    print("Premi 'l' per ATTERRARE")
    print("Premi 'q' per CHIUDERE tutto")

    try:
        while True:
            frame = cam.requestFrame(2000)
            if frame is not None and isinstance(frame, ac.DepthData):
                depth_buf = cv2.flip(frame.depth_data, 1)
                conf_buf = cv2.flip(getattr(frame, "confidence_data", None), 1)

                points = find_closest_points(depth_buf, conf_buf)
                
                # Feedback visivo
                norm = (depth_buf * (255.0 / MAX_DISTANCE)).astype(np.uint8)
                img = cv2.applyColorMap(norm, cv2.COLORMAP_RAINBOW)
                
                # Gestione input tastiera
                key = cv2.waitKey(1) & 0xFF
                if key == ord('y'):
                    correzione_attiva = True
                    print(">> MODO AUTONOMO: ON")
                elif key == ord('s'):
                    correzione_attiva = False
                    print(">> MODO AUTONOMO: OFF (Hovering)")
                elif key == ord('l'):
                    print(">> ATTERRAGGIO...")
                    vehicle.mode = VehicleMode("LAND")
                elif key == ord('q'):
                    break

                # Esecuzione correzione se attiva
                if correzione_attiva:
                    esegui_correzione_yaw(vehicle, points)
                    cv2.putText(img, "AUTONOMO: ON", (20, 40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                # Disegno punti rilevati
                for p in [pt for pt in points if pt]:
                    cv2.circle(img, (p['x'], p['y']), 15, (255, 255, 255), 2)
                
                cv2.imshow("Controllo Drone", img)
                cam.releaseFrame(frame)

    finally:
        vehicle.close()
        cam.stop()
        cam.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()