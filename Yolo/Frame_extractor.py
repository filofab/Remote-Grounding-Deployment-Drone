# python
import cv2
import os
import sys

def extract_frames(video_path, frame_interval, output_folder="frame"):
    """
    Estrae i frame da un video, salvando un frame ogni 'frame_interval' frame.
    """
    if frame_interval <= 0:
        print("Errore: 'frame_interval' deve essere un intero positivo.")
        return False

    if not os.path.exists(video_path):
        print(f"Errore: Il file '{video_path}' non esiste.")
        return False

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Errore: Impossibile aprire il video '{video_path}'")
        return False

    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    duration = total_frames / fps if fps > 0 else 0

    print(f"Video: {video_path}")
    print(f"FPS: {fps}")
    print(f"Frame totali: {total_frames}")
    print(f"Durata: {duration:.2f} secondi")
    print(f"Salverà un frame ogni {frame_interval} frame")

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
        print(f"Cartella '{output_folder}' creata.")
    else:
        print(f"Cartella '{output_folder}' già esistente.")

    frame_count = 0
    saved_count = 0

    print("\nEstrazione frame in corso...")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if frame_count % frame_interval == 0:
            frame_filename = os.path.join(output_folder, f"frame_{saved_count:06d}.jpg")
            cv2.imwrite(frame_filename, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            saved_count += 1
            if saved_count % 10 == 0:
                print(f"Salvati {saved_count} frame...")

        frame_count += 1

    cap.release()

    print(f"\nEstrazione completata!")
    print(f"Frame totali letti: {frame_count}")
    print(f"Frame salvati: {saved_count}")
    print(f"Frame salvati in: {os.path.abspath(output_folder)}")

    return True


def main():
    # Lettura argomenti / input e validazione
    if len(sys.argv) > 2:
        video_path = sys.argv[1]
        try:
            frame_interval = int(sys.argv[2])
        except ValueError:
            print("Errore: il secondo argomento deve essere un intero (frame_interval).")
            return
    elif len(sys.argv) > 1:
        video_path = sys.argv[1]
        while True:
            s = input("Ogni quanti frame: ")
            try:
                frame_interval = int(s)
                break
            except ValueError:
                print("Inserisci un intero valido.")
    else:
        video_path = input("Inserisci il percorso del file video: ")
        while True:
            s = input("Ogni quanti frame: ")
            try:
                frame_interval = int(s)
                break
            except ValueError:
                print("Inserisci un intero valido.")

    video_path = video_path.strip('"\'')

    extract_frames(video_path, frame_interval)


if __name__ == "__main__":
    main()