# Struttura versioni Progetto
Decrizione delle distinte versioni

## TEST
versione Beta per test interni



## Cartella V6
Versione di consolidamento sicurezza e refactoring (collaudo in volo da effettuare):
- Corretto import errato di time in util.py (send_velocity_body andava in crash con AttributeError)
- Lo sgancio (tasto d) è ora BLOCCATO se persona_detected è True e invia il comando reale al servo (canale N_SERV_SGANCIO, PWM_SGANCIO in config.py, da tarare sul meccanismo)
- Tutti i parametri di volo hardcoded spostati in config.py (distanze avvicinamento/ritiro, velocità, tempi di hover/stabilità/stazionamento, soglie rilevamento cavo)
- Rimossa la sleep bloccante nella salita del passo 2 (non blocca più preview e tastiera)
- Il reset (r) azzera anche lo stato di sgancio (prima il popup GUIDA SGANCIATA restava attivo)
- Il tasto y non incrementa più il token oltre il passo 4
- Rimosse variabili inutilizzate (mostra_popup, attesa_conferma2) e import ridondanti
- Aggiornato l'elenco comandi stampato all'avvio (aggiunti r, d, p)

## Cartella V5
Aggiunte logiche di sgancio e stazionamento sopra il cavo con controllo ogni 5 secondi, simulazione della persona identificata nel frame con tasto P (variabile True/False)



## Cartella V4
Versione V4 completa la posizione di stazionamento a cavoallo del cavo, 
e le correzioni esteriche e di logiche di reset e riavvio in volo 
collaudate con esito positivo

## Cartella V3
Backup_V3 fase finale, il drone raggiunge la posizione sopra il 
cavo rimanendo in volo stazionario

## Certella V2
Il codice consente la corretta esecuzione del passo 1 in tutte le
sue parti, mancano correzioni estetiche e di layout.

## Certella V1
Definizione dei file e struttura base del progetto

## Certella V0
Prima versione con 1 solo file, scartata

