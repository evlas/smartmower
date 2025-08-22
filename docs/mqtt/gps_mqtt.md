# Documentazione MQTT - Modulo GPS

## Panoramica
Gestisce la ricezione e l'elaborazione dei dati dal ricevitore GPS.

## Configurazione
- **File**: `/opt/smartmower/etc/config/robot_config.json`
- **Sezione**: `gps`

## Topic Pubblicati

### `smartmower/gps/data`
**Frequenza**: 1-5Hz  
**Formato**: JSON  
**Campi**:
- `latitude`: Latitudine in gradi decimali
- `longitude`: Longitudine in gradi decimali
- `altitude`: Altitudine in metri
- `speed`: Velocità in nodi
- `course`: Rotta in gradi
- `satellites`: Numero satelliti in vista
- `fix_quality`: Qualità del segnale (0-2)
- `hdop`: Precisione orizzontale
- `timestamp`: Timestamp UNIX in ms

### `smartmower/gps/status`
**Frequenza**: 1Hz  
**Contenuto**: Stato del ricevitore GPS
- `fix`: Tipo di fix (0=none, 1=2D, 2=3D)
- `satellites_used`: Satelliti utilizzati
- `hdop`: Precisione orizzontale
- `last_update`: Ultimo aggiornamento

## Topic Sottoscritti

### `smartmower/gps/commands`
**Comandi supportati**:
- `reset`: Riavvia il modulo GPS
- `cold_start`: Reset freddo
- `warm_start`: Reset caldo
- `hot_start`: Reset rapido

## Formato Messaggi

### Esempio Dati GPS
```json
{
  "timestamp": 1628671200000,
  "latitude": 45.4642,
  "longitude": 9.1900,
  "altitude": 120.5,
  "speed": 0.5,
  "course": 45.0,
  "satellites": 8,
  "fix_quality": 2,
  "hdop": 1.2
}
```

### Esempio Stato GPS
```json
{
  "timestamp": 1628671200000,
  "fix": 3,
  "satellites_used": 8,
  "hdop": 1.2,
  "last_update": "2023-08-11T11:20:00Z"
}
```
