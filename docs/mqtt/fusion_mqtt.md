# Documentazione MQTT - Modulo Fusion

## Panoramica
Fusione dati da IMU, GPS e odometria per la stima di posizione e orientamento.

## Configurazione
- **File**: `/opt/smartmower/etc/config/robot_config.json`
- **Sezione**: `fusion`

## Topic Pubblicati

### `smartmower/fusion/state`
**Frequenza**: 1-10Hz  
**Formato**: JSON  
**Campi**:
- `position`: Coordinate x,y,z in metri
- `orientation`: Quaternione (w,x,y,z)
- `velocity`: Velocità lineare e angolare
- `covariance`: Matrice di covarianza 6x6

### `smartmower/fusion/debug`
**Quando**: Su richiesta o debug  
**Contenuto**: Messaggi diagnostici

### `smartmower/fusion/error`
**Quando**: Errori  
**Campi**: Codice, messaggio, severità

## Topic Sottoscritti

### `smartmower/fusion/commands`
**Comandi supportati**:
- `reset_filter`: Azzera il filtro
- `set_parameters`: Configura parametri

### Dati Sensori
1. **IMU**: Accelerazione, velocità angolare, orientamento
2. **GPS**: Coordinate, velocità, qualità segnale
3. **Odometria**: Posizione e orientamento dalle ruote
