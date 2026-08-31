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
ROI_WIDTH_PX = 20                   #diametro della ROI in pixel
PERCENTILE_DISTANCE = 10            #percentile per il calcolo della distanza nella ROI

#RILEVAMENTO CAVO (passo 2, camera inclinata verso il basso)
CAVO_MIN_MM = 200                   #distanza minima valida per la ricerca del cavo
CAVO_MAX_MM = 2000                  #distanza massima valida per la ricerca del cavo
CAVO_SOGLIA_RILEVAMENTO_MM = 800    #sotto questa soglia la riga e' considerata cavo (da tarare)
CAVO_TOLLERANZA_CENTRO_PX = 20      #tolleranza in pixel per considerare il cavo al centro del frame


# ============================================================
# PARAMETRI DRONEKIT / CONTROLLO
# ============================================================

CONNECTION_STRING = "udpin:0.0.0.0:14550"          #stringa di connessione al drone
#PID E TOLLERANZE
GUADAGNO_YAW = 0.5                                 #guadagno per il controllo in yaw
SOGLIA_TOLLERANZA_MM = 30                          #tolleranza per considerare per il raggiungimento dell'obiettivo
SOGLIA_TOLLERANZA_PX = 50                           #
MAX_YAW_DEG = 15                                   #velocità angolare massima da inviare al drone
#REGOLE DI VOLO
TARGET_ALTITUDE_M = 5                              #Altezza di takeoff
INCREMENTO_ALTEZZA = 2                             #Quanto sopra l'ostacolo deve andare il drone

#PASSO 1 - ALLINEAMENTO E MANOVRA AVVICINAMENTO/RITIRO
TEMPO_STABILITA_S = 5.0                            #tempo in tolleranza per considerare stabile l'allineamento
DISTANZA_AVVICINAMENTO_MM = 400                    #distanza a cui fermarsi durante l'avvicinamento al cavo
DISTANZA_RITIRO_MM = 1000                          #distanza a cui fermarsi durante la retromarcia
VELOCITA_AVANZAMENTO_MS = 0.15                     #velocità di avanzamento/retromarcia (m/s)
TEMPO_HOVER_S = 3.0                                #durata dell'hover alla distanza di avvicinamento

#PASSO 2 - POSIZIONAMENTO SOPRA IL CAVO
VELOCITA_SALITA_MS = 0.15                          #velocità di salita (m/s)
VELOCITA_AVANTI_PASSO2_MS = 0.2                    #velocità di avanzamento per centrare il cavo (m/s)
TEMPO_STAZIONAMENTO_S = 5.0                        #tempo di verifica dello stazionamento a cavallo del cavo



# ============================================================
# PARAMETRI AUX
# ============================================================
PWM_TILT_CAM = 1900                                #valore PWM per l'inclinazione della camera TOF verso il basso
N_SERV_CAM = 9                                     #numero del canale AUX per il servo della camera TOF

#SGANCIO (meccanismo di rilascio del carico)
N_SERV_SGANCIO = 10                                #numero del canale AUX per il servo di sgancio (da configurare)
PWM_SGANCIO = 1900                                 #valore PWM per aprire il meccanismo di sgancio (da tarare)
