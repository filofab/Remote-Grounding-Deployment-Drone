#V2.1
# Modificato la ricerca in tutta la superfice delle colonne esterne della griglia
# Allargato il campo di ricerca a tuto lo schermo
# con un manico di scopa mantiene il "collegamento" ad una distanza inferiore ad 1m

import cv2
import numpy as np
import os
import ArducamDepthCamera as ac
import time

# ---------------- CONFIG ----------------
MAX_DISTANCE = 2000  # mm
PIXEL_WINDOW = 10  # diametro del cerchio interrogato (in pixel)
CROSS_SIZE = 30  # dimensione fissa della croce (metà ampiezza in pixel)

# Variabili parametriche
GRID_COLS = 3  # Numero di colonne della griglia (DEVE ESSERE MAGGIORE DI 2)
GRID_ROWS = 4  # Numero di righe della griglia

# Filtri di distanza (in mm)
MIN_DISTANCE = 200  # distanza minima (mm)
MAX_DISTANCE_FILTER = 4000  # distanza massima per il filtro (mm)
PHOTO_INTERVAL_SEC = 1  # intervallo in secondi
PATH_PHOTO = "Take_2.1_2m" # cartella di salvataggio delle foto

BUTTON_TOP_LEFT = (20, 20)
BUTTON_BOTTOM_RIGHT = (200, 70)
# ----------------------------------------

confidence_value = 120


def getPreviewRGB(preview: np.ndarray, confidence: np.ndarray) -> np.ndarray:
    """Rimuove NaN e applica soglia di confidence (se presente)."""
    preview = np.nan_to_num(preview)
    if confidence is not None:
        preview[confidence < confidence_value] = (0, 0, 0)
    return preview


def save_photo(image):
    """Salva in Take_1/photo_n.jpg con nome progressivo."""
    folder = PATH_PHOTO
    os.makedirs(folder, exist_ok=True)
    existing = [f for f in os.listdir(folder) if f.lower().endswith(".jpg")]
    photo_num = len(existing) + 1
    filename = os.path.join(folder, f"photo_{photo_num}.jpg")
    cv2.imwrite(filename, image)
    print(f"📸 Foto salvata in {filename}")


def draw_center_grid(image, rect_top_left, rect_bottom_right, rows=GRID_ROWS, cols=GRID_COLS, color=(200, 200, 200)):
    """Disegna un reticolo all'interno del rettangolo dato."""
    x1, y1 = rect_top_left
    x2, y2 = rect_bottom_right
    w = x2 - x1
    h = y2 - y1

    # contorno
    cv2.rectangle(image, (x1, y1), (x2, y2), color, 1)

    # righe verticali
    if cols > 1:
        step_x = w / cols
        for i in range(1, cols):
            xi = int(x1 + i * step_x)
            cv2.line(image, (xi, y1), (xi, y2), color, 1)

    # righe orizzontali
    if rows > 1:
        step_y = h / rows
        for j in range(1, rows):
            yj = int(y1 + j * step_y)
            cv2.line(image, (x1, yj), (x2, yj), color, 1)


def find_closest_points_in_outer_columns(depth_buf, confidence_buf, min_dist=MIN_DISTANCE, max_dist=MAX_DISTANCE_FILTER,
                                         cols=GRID_COLS):
    """
    Trova i punti più vicini nelle colonne esterne della griglia.
    Restituisce una lista di tuple (x, y, distanza) per ogni punto trovato.
    """
    if cols < 2:
        return []

    h, w = depth_buf.shape
    rect_w, rect_h = w , h
    x1_grid, y1_grid = (w - rect_w) // 2, (h - rect_h) // 2
    x2_grid, y2_grid = x1_grid + rect_w, y1_grid + rect_h

    points = []

    # Maschera di validità basata su distanza e confidenza
    valid_mask = (depth_buf > 0) & (depth_buf >= min_dist) & (depth_buf <= max_dist)
    if confidence_buf is not None:
        valid_mask &= (confidence_buf >= confidence_value)

    col_width = rect_w // cols

    # Ricerca nella prima colonna (esterna sinistra)
    x_start_left = x1_grid
    x_end_left = x_start_left + col_width
    col_mask_left = np.zeros_like(depth_buf, dtype=bool)
    col_mask_left[y1_grid:y2_grid, x_start_left:x_end_left] = True
    col_mask_left &= valid_mask

    if np.any(col_mask_left):
        depth_masked_left = np.where(col_mask_left, depth_buf, np.inf)
        flat_idx_left = np.argmin(depth_masked_left)
        y, x = np.unravel_index(flat_idx_left, depth_masked_left.shape)
        dist_mm = depth_buf[y, x]
        points.append({'x': x, 'y': y, 'dist': dist_mm})
    else:
        points.append(None)

    # Ricerca nell'ultima colonna (esterna destra)
    x_start_right = x1_grid + (cols - 1) * col_width
    x_end_right = x_start_right + col_width
    col_mask_right = np.zeros_like(depth_buf, dtype=bool)
    col_mask_right[y1_grid:y2_grid, x_start_right:x_end_right] = True
    col_mask_right &= valid_mask

    if np.any(col_mask_right):
        depth_masked_right = np.where(col_mask_right, depth_buf, np.inf)
        flat_idx_right = np.argmin(depth_masked_right)
        y, x = np.unravel_index(flat_idx_right, depth_masked_right.shape)
        dist_mm = depth_buf[y, x]
        points.append({'x': x, 'y': y, 'dist': dist_mm})
    else:
        points.append(None)

    return points


def draw_markers_and_line(image, points, window_size=PIXEL_WINDOW):
    """Disegna marcatori, distanze e una linea retta che unisce i punti trovati."""
    color = (255, 255, 255)  # Blu per i marcatori e la linea
    radius = window_size // 2

    valid_points = [p for p in points if p is not None]

    if len(valid_points) > 0:
        # Disegna i marcatori e le distanze per ogni punto
        for p in valid_points:
            x, y, dist = p['x'], p['y'], p['dist']
            cv2.circle(image, (x, y), radius, color, 2)
            cv2.line(image, (x - CROSS_SIZE, y), (x + CROSS_SIZE, y), color, 5)
            cv2.line(image, (x, y - CROSS_SIZE), (x, y + CROSS_SIZE), color, 5)
            cv2.putText(image, f"{dist / 10:.1f} cm", (x + 15, y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Disegna la linea retta che unisce i punti
        for i in range(len(valid_points) - 1):
            cv2.line(image, (valid_points[i]['x'], valid_points[i]['y']),
                     (valid_points[i + 1]['x'], valid_points[i + 1]['y']), color, 5)

        # Calcola la distanza media
        mean_dist_mm = np.mean([p['dist'] for p in valid_points])
        mean_dist_text = f"Media: {mean_dist_mm / 10:.1f} cm"
    else:
        mean_dist_text = "Media: -- cm"

    # Testo in basso a sinistra
    h, w, _ = image.shape
    text1 = f"N pixel = {window_size}"
    text2 = f"Distanza = {mean_dist_text}"

    margin = 20
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.7
    thickness = 2
    color_text = (255, 255, 255)

    cv2.putText(image, text1, (margin, h - margin - 25), font, scale, color_text, thickness, cv2.LINE_AA)
    cv2.putText(image, text2, (margin, h - margin), font, scale, color_text, thickness, cv2.LINE_AA)


def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        preview_img = param
        if preview_img is not None:
            if (BUTTON_TOP_LEFT[0] <= x <= BUTTON_BOTTOM_RIGHT[0] and
                    BUTTON_TOP_LEFT[1] <= y <= BUTTON_BOTTOM_RIGHT[1]):
                save_photo(preview_img)


def main():
    global confidence_value, depth_buf


    cam = ac.ArducamCamera()
    ret = cam.open(ac.Connection.CSI, 0)
    if ret != 0:
        print("Failed to open camera. Error code:", ret)
        return
    ret = cam.start(ac.FrameType.DEPTH)
    if ret != 0:
        print("Failed to start camera. Error code:", ret)
        cam.close()
        return

    cam.setControl(ac.Control.RANGE, MAX_DISTANCE)
    r = cam.getControl(ac.Control.RANGE)
    info = cam.getCameraInfo()
    print(f"Camera resolution: {info.width}x{info.height}")

    cv2.namedWindow("preview", cv2.WINDOW_AUTOSIZE)
    if info.device_type == ac.DeviceType.VGA:
        cv2.createTrackbar("confidence", "preview", confidence_value, 255,
                           lambda v: globals().update(confidence_value=v))
    cv2.setMouseCallback("preview", on_mouse, param=None)

    last_photo_time = time.time()

    while True:
        frame = cam.requestFrame(2000)
        preview_img = None
        if frame is not None and isinstance(frame, ac.DepthData):
            depth_buf = frame.depth_data
            confidence_buf = getattr(frame, "confidence_data", None)

            # Flip dei dati depth e confidence prima di elaborarli
            depth_buf = cv2.flip(depth_buf, 1)
            if confidence_buf is not None:
                confidence_buf = cv2.flip(confidence_buf, 1)

            # 1. Crea una copia per la visualizzazione
            depth_buf_display = np.copy(depth_buf)
            # 2. Imposta a zero i valori oltre la soglia di 3000 mm
            depth_buf_display[depth_buf_display > MAX_DISTANCE_FILTER] = 0

            # 3. Normalizza l'immagine di visualizzazione
            norm = (depth_buf_display * (255.0 / max(r, 1))).astype(np.uint8)
            # 4. Applica la mappa di colori
            color_img = cv2.applyColorMap(norm, cv2.COLORMAP_RAINBOW)

            # 5. Crea la maschera per i pixel che devono essere rossi
            zero_depth_mask = (depth_buf_display == 0)

            # 6. Applica il colore rosso GBR (255, 34, 0) ai pixel mascherati.
            # Converti in BGR: (0, 34, 255)
            color_img[zero_depth_mask] = (0, 34, 255)

            # 7. Prepara l'immagine finale per la visualizzazione
            preview_img = getPreviewRGB(color_img.copy(), confidence_buf)

            # Disegno reticolo
            h, w = depth_buf.shape
            rect_w, rect_h = w , h
            rect_x1, rect_y1 = (w - rect_w) // 2, (h - rect_h) // 2
            rect_x2, rect_y2 = rect_x1 + rect_w, rect_y1 + rect_h
            draw_center_grid(preview_img, (rect_x1, rect_y1), (rect_x2, rect_y2))

            # Trova i due punti e le loro distanze, poi disegna i marcatori e la linea
            points = find_closest_points_in_outer_columns(depth_buf, confidence_buf)
            draw_markers_and_line(preview_img, points, PIXEL_WINDOW)

            # Foto scatto continuo
            key = cv2.waitKey(1) & 0xFF
            current_time = time.time()
            if current_time - last_photo_time >= PHOTO_INTERVAL_SEC:
                if preview_img is not None:
                    save_photo(preview_img)
                    last_photo_time = current_time
            if key == ord("q"):
                break
            elif key == 32 or key == ord("t"):
                if preview_img is not None:
                    save_photo(preview_img)

            # pulsante
            cv2.rectangle(preview_img, BUTTON_TOP_LEFT, BUTTON_BOTTOM_RIGHT, (50, 50, 255), -1)
            cv2.putText(preview_img, "Take a photo", (BUTTON_TOP_LEFT[0] + 10, BUTTON_TOP_LEFT[1] + 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            cv2.setMouseCallback("preview", on_mouse, param=preview_img)
            cv2.imshow("preview", preview_img)

            cam.releaseFrame(frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == 32 or key == ord("t"):
            if preview_img is not None:
                save_photo(preview_img)

    cam.stop()
    cam.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()