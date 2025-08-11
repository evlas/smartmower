# Modulo di Fusione Sensori GPS/IMU

Questo modulo si occupa di gestire i dati provenienti dai sensori GPS e IMU, elaborarli e pubblicarli tramite MQTT.

## Struttura MQTT

### Topic
- `smartmower/sensors/fusion/velocity` - Dati di velocità del robot
- `smartmower/sensors/fusion/position` - Dati di posizione GPS
- `smartmower/sensors/fusion/imu` - Dati IMU grezzi
- `smartmower/sensors/fusion/status` - Stato del sensore

### Formato dei Messaggi
Vedere i file di esempio nella cartella `examples/`.

## Configurazione
Modificare `config.json` per impostare i parametri di connessione MQTT e i parametri dei sensori.

## Compilazione
```bash
make
```

## Esecuzione
```bash
./fusion_sensor
```
