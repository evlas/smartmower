# Firmware Pico - Telemetria, Errori e Protocollo

Questo documento descrive il comportamento del firmware del Raspberry Pi Pico in `firmware/main.py`, il protocollo di comunicazione, i bit di evento e i valori sentinella usati quando un componente non è inizializzato o non disponibile.

## UART e framing
- UART0: `TX=GP0`, `RX=GP1`, 115200 8N1.
- Framing: COBS + CRC16-CCITT (polinomio 0x1021, init 0xFFFF).
- Ogni frame termina con byte `0x00`.

## Messaggi pubblicati (Pico → Host)
- `MSG_ODOM (0x02)`: 6 x float32 LE → `x, y, theta, vx, vy, vtheta`
- `MSG_SONAR (0x03)`: 3 x float32 LE → `left_m, center_m, right_m`
- `MSG_IMU (0x01)`: 10 x float32 LE → `qw, qx, qy, qz, ax, ay, az, gx, gy, gz`
- `MSG_BATT (0x04)`: 2 x float32 LE → `voltage_V, current_A`
- `MSG_BLADES_RPM (0x06)`: 2 x float32 LE → `blade1_rpm, blade2_rpm`
- `MSG_EVENT (0x05)`: 16-bit LE bitfield di eventi/stato/errore

I rate di pubblicazione sono configurati in `pins_config.py`:
- `ODOM_RATE_HZ`, `SONAR_RATE_HZ`, `IMU_RATE_HZ`, `BATT_RATE_HZ`, più il calcolo RPM lame.

## Messaggi di comando (Host → Pico)
- `MSG_CMD_DRIVE (0x10)`: 2 x float32 LE → `left, right` in [-1..1]
- `MSG_CMD_BLADES (0x11)`: 2 x float32 LE → `blade1, blade2` in [-1..1]
- `MSG_CMD_RELAY (0x12)`: 1 x uint8 → `0`=disable, `1`=enable
- `MSG_CMD_LIMITS (0x13)`: 2 x float32 LE → `max_abs_speed, accel_limit`

## Eventi (`MSG_EVENT` bitfield)
Bit definiti in `main.py` (sorgente di verità):
- Stato/sensori
  - `EVENT_RELAY_ENABLED`   = 1<<0
  - `EVENT_BUMPER_LEFT`     = 1<<1
  - `EVENT_BUMPER_RIGHT`    = 1<<2
  - `EVENT_LIFT`            = 1<<3
  - `EVENT_RAIN`            = 1<<4
  - `EVENT_AUX1`            = 1<<5
  - `EVENT_AUX2`            = 1<<6
  - `EVENT_AUX3`            = 1<<7
  - `EVENT_PERIMETER_LEFT`  = 1<<8
  - `EVENT_PERIMETER_RIGHT` = 1<<9
  - `EVENT_TILT`            = 1<<11  (AUX4 mappato a TILT)
- Errori/diagnostica
  - `EVENT_ERR_PCF`   = 1<<10
  - `EVENT_ERR_IMU`   = 1<<12
  - `EVENT_ERR_BATT`  = 1<<13
  - `EVENT_ERR_SONAR` = 1<<14
  - `EVENT_ERR_ODOM`  = 1<<15

All'avvio, il firmware invia un `MSG_EVENT` con la maschera errori per segnalare quali moduli non sono disponibili. Nota: non è previsto un bit HEARTBEAT dedicato.

## Valori sentinella quando un modulo non è disponibile
Quando un componente non è inizializzato o non disponibile, il firmware continua a pubblicare telemetria usando valori sentinella per indicare la condizione d'errore:
- `ODOM` → `x, y, theta, vx, vy, vtheta = NaN`
- `SONAR` → `left, center, right = -1.0`
- `IMU` → tutti i 10 campi `NaN`
- `BATT` → `voltage_V = -1.0`, `current_A = 0.0`
- `BLADES_RPM` → se non disponibili, potrebbe essere omesso o inviato come `NaN, NaN` (a seconda della configurazione)

In parallelo, viene inviato un `MSG_EVENT` con i bit `EVENT_ERR_*` coerenti al modulo assente.

## LED di stato UART (Pico)
- LED onboard (GP25): si accende su attività RX/TX UART e si spegne dopo `UART_LED_TIMEOUT_MS` senza traffico.

## Dipendenze e moduli
Il firmware dipende da moduli in `firmware/app/`:
- `protocol.py` (framing COBS, CRC, IDs messaggi)
- `drive.py`, `blades.py`, `safety.py` (attuatori)
- `encoder.py`, `odometry.py` (odometria)
- `adafruit_bno055.py` (IMU), `ina226_batt.py` (batteria)
- `sonar_ultrasonic.py` (sonar)
- `pcf8574.py`, `bumper_sensors.py`, `lift_sensors.py`, `perimeter_sensors.py`
- Configurazione pin in `pins_config.py`

## Strategie di debug
- Abilitare i log tramite `_dbg(...)` in `main.py` (controllati da `DEBUG_INIT`/`DEBUG_STATUS_RATE_MS`).
- Verificare i flag `*_ok` e i bit `EVENT_ERR_*` per individuare moduli non operativi.
- Usare i valori sentinella per distinguere tra "dato valido" e "modulo non disponibile" lato host.

## Compatibilità
- Il comportamento è retro-compatibile con il protocollo esistente: i payload non cambiano formato; solo i valori possono essere sentinelle.
- Gli eventi includono nuovi bit: gli host che non li decodificano possono comunque leggere la maschera grezza.

## Esempi lato host (D1 Mini)
- Stampare la maschera eventi grezza: `EVENT RAW=0xXXXX`.
- Interpretare i bit simbolici (in `printEventBits()` sono inclusi i nuovi errori).
- Trattare `NaN`/`-1.0` come indicazione di modulo assente o errore.

