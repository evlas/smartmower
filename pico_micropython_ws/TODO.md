# TODO – Odometry/EKF integrazione (Pico + ROS)

## Obiettivo
- **Separare i ruoli**: il Pico invia solo misure istantanee (velocità o tick), mentre **la posa (x, y, θ)** è **integrata esclusivamente** dall’EKF sul Raspberry Pi (fusione GPS + IMU + Odom).

## Stato attuale (da modificare)
- `MSG_ODOM (0x02)` invia 6×`float32` LE: `x, y, th, vx, vy(=0), vth` da `pico_micropython_ws/firmware/main.py` (pack `'<ffffff'`).
- Questo implica integrazione della posa a bordo Pico, che va rimossa per utilizzo con EKF ROS.

## Decisione architetturale
- **EKF lato Raspberry Pi** è l’unica fonte di verità per `x, y, θ`.
- **Pico** invia solo misure istantanee per l’odometria ruote.

## Payload consigliato per MSG_ODOM (0x02)
- **Opzione A (consigliata)**: delta tick + dt
  - Formato: `'<ii f'`
  - Campi:
    - `Δticks_left: int32`
    - `Δticks_right: int32`
    - `dt: float32` (secondi)
  - Pro: massima fedeltà, diagnostica migliore, derivazione `vx/vth` lato ROS con parametri ruota.

  - Importante: i conteggi `Δticks` devono essere INTERI CON SEGNO (firmati) usando il segno dal pin `DIR` per indicare il verso di rotazione.

  Specifica campi (Opzione A):

  | Campo | Tipo | Unità | Descrizione |
  |---|---|---|---|
  | Δticks_left | int32 | conteggi | Variazione netta encoder ruota sinistra in Δt (con segno) |
  | Δticks_right | int32 | conteggi | Variazione netta encoder ruota destra in Δt (con segno) |
  | dt | float32 | secondi | Intervallo di tempo esatto dall’ultimo invio |

- **Opzione B**: velocità istantanee
  - Formato: `'<ff'`
  - Campi:
    - `vx: float32` [m/s]
    - `vth: float32` [rad/s]
  - Pro: semplice; Contro: perdita info tick.

- In entrambi i casi: **non inviare** `x, y, th` dal Pico.

## Frequenze e timing
- **Frequenza odom**: 10 Hz.
  - Motivazione: con encoder da 12 impulsi/giro motore e riduzione 185:1, a basse velocità la risoluzione a 100 Hz è insufficiente; 10 Hz fornisce Δticks più robusti e informativi.
  - Header frame (invariato): `[msg_id:1][len:1][seq:1][ts_ms:uint32][payload][crc16]` con COBS + `0x00` terminatore.
  - Usare `ts_ms` per time-sync lato ROS.

## Modifiche firmware (Pico)
- File: `pico_micropython_ws/firmware/main.py`
  - Blocco “Odometry publish loop”:
    - Continuare a leggere: `tL = enc_left.read_and_reset()`, `tR = enc_right.read_and_reset()`
    - Calcolare `dt` come già fatto.
    - Opzione A: `payload = ustruct.pack('<ii f', tL*sL, tR*sR, dt)`
    - Opzione B: derivare `v = d/dt`, `vth = dth/dt` e inviare `'<ff'`.
    - Rimuovere dal payload: `x, y, th, vy` e l’integrazione locale della posa.
  - Aggiornare `len` coerentemente (12 byte per A, 8 byte per B).
- File: `pico_micropython_ws/firmware/app/protocol.py`
  - Mantenere `MSG_ODOM = 0x02` ma **aggiornare la documentazione** del payload.

## Nodo ROS (Raspberry Pi) – robot_localization
- Sorgente “wheel odom” pubblicata come `geometry_msgs/TwistWithCovarianceStamped`:
  - `twist.twist.linear.x = vx`
  - `twist.twist.angular.z = vth`
  - Impostare covarianze realistiche.
- Se si usa **Opzione A** (tick):
  - Nodo ROS converte `Δticks` + `dt` in `vx` e `vth` con parametri:
    - `wheel_radius_m`, `wheel_base_m`, `ticks_per_rev_motor * gear_ratio`.
  - Usa `ts_ms` del Pico per timestamp.
- Fusioni EKF:
  - IMU (`sensor_msgs/Imu`): gyro/acc (eventuale yaw da mag se affidabile).
  - GPS tramite `navsat_transform_node` → `PoseWithCovarianceStamped`.
  - Odom ruote come sopra.

## Compatibilità e note
- Cambiando il payload, il valore `len` nel frame cambia: aggiornare il parser lato ROS.
- Mantenere gli ID messaggi invariati per backward-compat se serve, ma documentare chiaramente la **nuova semantica** di `MSG_ODOM`.
- Inviare `NaN` o flag evento in caso di sensori non validi (coerente con gestione attuale degli errori).

## Checklist
- [x] Confermata scelta payload `MSG_ODOM`: **Opzione A (Δticks + dt, firmati via DIR)**
- [ ] Impostare frequenza odometria a **10 Hz** (`ODOM_RATE_HZ = 10` in `app/pins_config.py`)
- [ ] Implementare modifica in `firmware/main.py` (rimozione posa, nuovo payload)
- [ ] Aggiornare documentazione in `app/protocol.py` e `README.md`
- [ ] Implementare/parlare il nodo ROS per parsing seriale e pubblicazione Twist
- [ ] Configurare `robot_localization` (covarianze, topic, frame_id, sincronizzazione)
- [ ] Test su banco: coerenza velocità, latenza, time-stamp
- [ ] Test in campo: drift EKF, comportamento con GPS e IMU
