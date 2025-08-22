# Safety Supervisor — MQTT

Percorso sorgenti: `src/safety/`
Eseguibile: `src/safety/bin/safety_supervisor`

## Config rilevante
- `mqtt.{broker,port,username,password,root_topic}` (chiave: `mqtt.root_topic`)
- `mqtt.topics.pico.{base,subtopics.data,qos}`
- `mqtt.topics.safety.{base,subtopics.state,subtopics.estop,qos,retain}`
- `system.safety.emergency_stop_enabled`
- `system.safety.battery.{low_threshold,type,profiles}`

## Topic sottoscritti (input)

- `<root>/<mqtt.topics.pico.base>/<mqtt.topics.pico.subtopics.data>`
  - Default: `smartmower/bridge/pico/data`
  - Scopo: ricevere telemetria e flag di sicurezza dal bridge Pico
  - Payload (JSON) — campi rilevanti:
    - E‑Stop: preferito `safety.emergency: bool`; compatibilità: `estop|estop_pressed: bool`
    - Batteria:
      - Tensione: `power.bus_voltage: number` (preferito) o `voltage_v: number`
      - Corrente: `power.current: number` (A; segno <0 carica, >0 scarica)
      - Percentuale grezza (fallback): `battery_pct: number`
    - Lift: `safety.lift: bool` (se sensore abilitato)

- `<root>/<mqtt.topics.gps.base>/<mqtt.topics.gps.subtopics.data>`
  - Default: `smartmower/bridge/gps/data`
  - Scopo: geofence
  - Payload (JSON): `latitude: number`, `longitude: number`

## Topic pubblicati (output)

- `<root>/<mqtt.topics.safety.base>/<mqtt.topics.safety.subtopics.estop>`
  - Default: `smartmower/safety/estop`
  - Payload: booleano semplice `true|false`
  - Note: QoS/retain da `mqtt.topics.safety.{qos,retain}`
  - Quando `tilt:true` viene ricevuto dal Pico e `system.safety.emergency_stop_enabled=true`, l'E‑Stop viene latcheato a `true` e viene inviato un comando al Pico su `.../bridge/pico/commands/emergency_stop` con `{"reason":"tilt"}`.

- `<root>/<mqtt.topics.safety.base>/<mqtt.topics.safety.subtopics.state>`
  - Default: `smartmower/safety/state`
  - Payload (JSON):
    ```json
    {
      "battery_voltage_v": <number>,
      "battery_percentage": <number>,
      "battery_state": "charge" | "discharge",
      "battery_low": true | false,
      "lift": true | false,
      "theft_alarm": true | false,
      "tilt": true | false
    }
    ```
  - Calcoli:
    - `battery_voltage_v`: da `power.bus_voltage` o `voltage_v`.
    - `battery_percentage`: stimata via profilo OCV `system.safety.battery.profiles.<type>.ocv_to_pct` (interpolazione lineare). Fallback a `battery_pct` se profilo o tensione assenti.
    - `battery_state`: da `power.current` (A). `current<0 → charge`, altrimenti `discharge`.
    - `battery_low`: confronta `battery_percentage` con `system.safety.battery.low_threshold`.
    - `lift`: letto da `safety.lift` del payload Pico (se abilitato).
    - `theft_alarm`: attivo se (lift==true) oppure (fuori geofence) quando `theft_alarm_enabled`. È latching finché non arriva un reset utente su `smartmower/safety/commands`.

## Note di implementazione
- L’E‑Stop è applicato solo se `system.safety.emergency_stop_enabled` è `true`.
- Subscribe al topic Pico con QoS `mqtt.topics.pico.qos`.
- Publish dei topic safety con `mqtt.topics.safety.qos` e `mqtt.topics.safety.retain`.
- La separazione Bridge↔Safety mantiene i topic di output stabili anche se cambia il payload del bridge.

## Geofence
- Config opzionale: `system.safety.geofence.polygon` come array di punti `[lat, lon]` (almeno 3) che definisce un poligono (non serve convesso). Viene usato ray‑casting per il test punto‑in‑poligono.
- Se non presente o con meno di 3 punti, il geofence è considerato disabilitato (interno sempre true).
- Uscire dal poligono imposta `theft_alarm=true` (latching). Reset con:
  ```bash
  mosquitto_pub -t smartmower/safety/commands -m '{"reset":{"theft_alarm":true}}'
  ```
