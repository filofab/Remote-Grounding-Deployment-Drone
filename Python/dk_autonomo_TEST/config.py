import cv2
# ============================================================
# PARAMETRI DI FLIP IMMAGINE E TESTO
# ============================================================

FLIP_HORIZONTAL = True              #se True, capovolge l'immagine orizzontalmente
FLIP_VERTICAL = False               #se True, capovolge l'immagine verticalmente
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE_TITLE = 0.9
FONT_SCALE_CMD = 0.8
FONT_THICK = 2
PADDING = 10




# ============================================================
# PARAMETRI TOF / VISIONE
# ============================================================

MAX_DISTANCE = 4000                 #distanza massima in mm del sensore TOF
MIN_DISTANCE_MM = 200               #distanza minima in mm per il pilota automatico
MAX_DISTANCE_MM = 2000              #distanza massima in mm per il pilota automatico

CONFIDENCE_THRESHOLD = 30           #soglia di confidenza per il filtro dei punti

VERTICAL_LINE_SPACING_PX = 120      #distanza tra le linee verticali in pixel
ROI_WIDTH_PX = 20                   #larghezza della ROI in pixel
PERCENTILE_DISTANCE = 10            #percentile per il calcolo della distanza nella ROI


# ============================================================
# PARAMETRI DRONEKIT / CONTROLLO
# ============================================================

CONNECTION_STRING = "udpin:0.0.0.0:14550"          #stringa di connessione al drone
#PID E TOLLERANZE
GUADAGNO_YAW = 0.5                                 #guadagno per il controllo in yaw
SOGLIA_TOLLERANZA_MM = 30                          #tolleranza per considerare per il raggiungimento dell'obiettivo
MAX_YAW_DEG = 15                                   #velocità angolare massima da inviare al drone
#REGOLE DI VOLO
TARGET_ALTITUDE_M = 5                              #Altezza di takeoff
INCREMENTO_ALTEZZA = 2                             #Quanto sopra l'ostacolo deve andare il drone



# ============================================================
# PARAMETRI AUX
# ============================================================
PWM_TILT_CAM = 1900                                #valore PWM per l'inclinazione della camera TOF verso il basso
N_SERV_CAM = 9                                     #numero del canale AUX per il servo della camera TOF



