# Remote-Grounding-Deployment-Drone
A Remote Grounding Deployment Drone is a specialized unmanned aerial system (UAS) designed to safely install grounding or earthing cables on live, high-voltage transmission lines




### Passi volo con Dronekit

## Posizionamento
Il drone va posizionato difronte al cavo AT, alla stessa quota

## Passo 1 Avvio Python

1. Posizionamento autonomo perpendicolare al cavo AT
2. Tocco con Sensore di presenza corrente/tensione = Si avvicina ad una distanza di 1m dal cavo e si riallontana
3. Il codice chiede la conferma per fare il passo successivo

## Passo 2 Posizionamento a cavallo del cavo AT

1. Si solleve di 1m inclina la Camera in posizione Azimutale verso il basso (Aux 6 finzo servomotore)
2. Avanza e con la TOF controlla con una misura lungo una linea verticale quando si trova sopra il cavo (= al centro dell'inquadratura)
3. rotazione di 90° in senso orario/antiorario per posizionarsi a cavallo del cavo AT

## Passo 3

1.  Fotografia con IA -> riconoscimento presenza o meno persone o animali
2. Consenso operatore sgancio




