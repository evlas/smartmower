# Documentazione MQTT - Modulo PICO

## Panoramica
Il modulo PICO è un bridge che gestisce la comunicazione tra i componenti hardware di basso livello (IMU, motori, sensori) e il sistema di controllo centrale. Utilizza un protocollo binario ottimizzato per la comunicazione in tempo reale.

## Configurazione
- **File di configurazione**: `/opt/smartmower/etc/config/robot_config.json`
 - **Chiavi rilevanti per i topic**:
   - `mqtt.root_topic` (default: `smartmower`)
   - `mqtt.topics.pico.base` (default: `bridge/pico`)

## Struttura dei Topic

### Topic Base
Il topic base è composto come segue (valori presi dalla configurazione):
```
<base_topic> = <mqtt.root_topic>/<mqtt.topics.pico.base>
```
Esempio con default: `smartmower/bridge/pico`.

## Topic Pubblicati

### `<base_topic>/data`
**Tipo**: Pubblicazione
**Formato**: JSON
**Descrizione**: Dati sensori aggregati e flag di sicurezza. I dati binari dal firmware vengono convertiti in JSON dal bridge (vedi `src/bridge/pico/src/pico/pico_interface.cpp::publishSensorData()`).
**Campi**:
- `timestamp` (uint32, ms)
- `accel` {`x`,`y`,`z`} [m/s^2]
- `gyro` {`x`,`y`,`z`} [rad/s]
- `mag` {`x`,`y`,`z`} [uT]
- `ultrasonic` {`left`,`center`,`right`} [m]
- `power` {`bus_voltage` [V], `current` [A]}
- `safety` {`emergency`, `rain`, `bumper`, `lift`, `tilt`} (bool)
**Esempio**:
```json
{
  "timestamp": 1628671200000,
  "accel": {"x": 0.01, "y": -0.02, "z": 9.80},
  "gyro": {"x": 0.001, "y": 0.000, "z": -0.002},
  "mag": {"x": 20.5, "y": -5.3, "z": 42.0},
  "ultrasonic": {"left": 0.35, "center": 0.50, "right": 0.33},
  "power": {"bus_voltage": 24.1, "current": 3.2},
  "safety": {"emergency": false, "rain": false, "bumper": false, "lift": false, "tilt": false}
}
```

### `<base_topic>/status`
**Tipo**: Pubblicazione
**Formato**: JSON
**Descrizione**: Report di stato con informazioni su motori, flag di sistema e relè (vedi `src/bridge/pico/src/pico/pico_interface.cpp::publishStatusReport()`).
**Campi**:
- `timestamp` (uint32, ms)
- `motors`: array[4] di oggetti `{ speed, rpm, encoder }`
- `system_flags` (uint8) — bit0: `emergency`, bit1: `calibrated`
- `relay_state` (bool)
**Esempio**:
```json
{
  "timestamp": 1628671200000,
  "motors": [
    {"speed": 0.10, "rpm": 95.0, "encoder": 12345},
    {"speed": 0.10, "rpm": 96.0, "encoder": 12344},
    {"speed": 0.00, "rpm": 0.0,  "encoder": 0},
    {"speed": 0.00, "rpm": 0.0,  "encoder": 0}
  ],
  "system_flags": 0,
  "relay_state": false
}
```

## Topic Sottoscritti

### `<base_topic>/commands/#`
**Tipo**: Sottoscrizione wildcard
**Formato**: JSON
**Descrizione**: Comandi inoltrati via UART al firmware Pico.

#### Comando motori
**Topic**: `<base_topic>/commands/motors`
**Payload**:
- `left` (float, [-1.0, 1.0])
- `right` (float, [-1.0, 1.0])
- `blade1` (float, [0.0, 1.0])
- `blade2` (float, [0.0, 1.0])
**Esempio**:
```json
{ "left": 0.5, "right": 0.5, "blade1": 0.0, "blade2": 0.0 }
```

#### Comando di sistema
**Topic**: `<base_topic>/commands/system`
**Payload**:
- `action`: "emergency_stop" | "estop_reset" | "set_relay" | "calibrate" | "reset_encoders"  
  Nota: per compatibilità è accettato anche `reset`, equivalente a `estop_reset`.
- `value`: float (obbligatorio per `set_relay`, 0.0=OFF, 1.0=ON; ignorato per gli altri)
**Esempi**:
```json
{ "action": "emergency_stop" }
```
```json
{ "action": "estop_reset" }
```
```json
{ "action": "set_relay", "value": 1.0 }
```
```json
{ "action": "calibrate" }
```
```json
{ "action": "reset_encoders" }
```

## Note Importanti
1. Le unità e i campi seguono la definizione delle strutture in `src/bridge/pico/include/pico/pico_protocol.h`.
2. La valutazione del flag `safety.tilt` è calcolata nel bridge a partire da `accel` usando la soglia `sensors.tilt_angle_limit` (gradi) in configurazione.
3. I topic sono composti usando `mqtt.root_topic` e `mqtt.topics.pico.base` (vedi `src/config/robot_config.json`).
4. Mappa ID comandi di sistema (allineata):
   - 0x01 = `EMERGENCY_STOP`
   - 0x02 = `RESET`
   - 0x03 = `SET_RELAY` (usa `value` 0.0/1.0)
   - 0x04 = `CALIBRATE` (calibrazione IMU/Magnetometro; implementata, flag `system_flags` bit1)
   - 0x05 = `RESET_ENCODERS`

## Note Tecniche
- Il modulo utilizza la libreria Mosquitto per la comunicazione MQTT
- La connessione MQTT viene gestita automaticamente con meccanismi di riconnessione in caso di problemi
- I messaggi in formato JSON devono essere ben formati e validi

## Integrazione con Safety
- Il servizio Safety consuma `<root>/<mqtt.topics.pico.base>/data` e legge in particolare:
  - `safety.emergency` (boolean) per E‑Stop
  - `power.bus_voltage` (V) e `power.current` (A) per lo stato batteria
  - Fallback compatibilità: `voltage_v` e `battery_pct`
- Per lo schema di output di Safety, vedi `docs/mqtt/safety_mqtt.md`.
