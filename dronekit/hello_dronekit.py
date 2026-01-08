from dronekit import connect

# udpin:0.0.0.0:14550 significa: resta in ascolto su tutte le 
# interfacce del Raspberry sulla porta 14550
connection_string = 'udpin:0.0.0.0:14550'

print(f"In attesa di connessione da SITL su {connection_string}...")

try:
    vehicle = connect(connection_string, wait_ready=True, timeout=120)
    print("\n--- Connessione Stabilita! ---")
    print(f" GPS: {vehicle.gps_0}")
    print(f" Battery: {vehicle.battery}")
    print(f" Mode: {vehicle.mode.name}")
    vehicle.close()
except Exception as e:
    print(f"Errore durante la connessione: {e}")