# State Machine — MQTT

Questo documento elenca i topic MQTT pubblicati e sottoscritti dal modulo `state_machine` e definisce i payload JSON/flat supportati.

Percorso sorgenti: `src/state_machine/`

## Config rilevante
- `mqtt.{broker,port,username,password,root_topic}`
- `mqtt.topics.{mission,safety}` per derivare i topic completi
- Pubblicazione status: `smartmower/status/{state|event|transition}` (prefisso fisso)

## Topic sottoscritti

- `smartmower/mission/state`
  - Scopo: ricevere intenzioni/commandi di missione da Web Admin o Mission Manager
  - Formati payload accettati:
    - Testo semplice: `"start"`, `"manual"`
    - JSON equivalenti:
      - `{ "state": "start" }`
      - `{ "cmd": "start" }`
      - `{ "manual": true }`
  - Effetto:
    - `start` -> genera evento `START_MOWING`
    - `manual` -> genera evento `MANUAL_CONTROL`

- `smartmower/safety/state`
  - Scopo: ricevere stato sintetico di sicurezza (aggregato)
  - Payload JSON:
    - `{ "battery_low": true|false, ... }`
  - Effetto:
    - `battery_low==true` -> genera evento `BATTERY_LOW`

- `smartmower/safety/estop`
  - Scopo: ricevere stato del pulsante E‑Stop aggregato
  - Formati payload accettati:
    - Testo semplice: `"pressed"`, `"true"`, `"1"`, `"estop"`
    - JSON equivalenti:
      - `{ "pressed": true }`
      - `{ "estop": true }`
      - `{ "state": "pressed" }`
  - Effetto:
    - genera evento `EMERGENCY_STOP`

### Comandi Manuali (topic unificato)

- `smartmower/state_machine/commands` (o override via `state_machine.command_topic`)
  - Scopo: inviare comandi di controllo manuale quando lo stato corrente è `MANUAL_CONTROL`.
  - Payload JSON supportati:
    - Impostazione velocità:
      ```json
      { "manual": { "cmd_vel": { "linear": 0.30, "angular": 0.10 } } }
      ```
    - Comando lame ON/OFF:
      ```json
      { "manual": { "blade": { "on": true } } }
      ```
    - Uscita da manuale (→ `END_MANUAL_CONTROL`):
      ```json
      { "manual": { "exit": true } }
      ```
      Alternativa equivalente:
      ```json
      { "manual": { "resume": true } }
      ```
    - Comando combinato (velocità + lame):
      ```json
      { "manual": { "cmd_vel": { "linear": 0.2, "angular": -0.3 }, "blade": { "on": false } } }
      ```
  - Note:
    - I topic legacy `smartmower/manual/{cmd_vel,blade_cmd,exit}` restano accettati per retrocompatibilità.
    - L’ingresso in manuale si richiede pubblicando `"manual"` (stringa) sullo stesso topic comandi.

## Topic pubblicati

- `smartmower/status/state`
  - Scopo: stato corrente della macchina a stati
  - Payload JSON (ricco):
    ```json
    {
      "current": "IDLE",
      "previous": "INIT",
      "last_event": "INIT_COMPLETE",
      "last_reason": "Init completed",
      "time_in_state_sec": 12.3,
      "tick_count": 123,
      "timestamp": "2025-08-17T20:00:00+02:00"
    }
    ```

- `smartmower/status/event`
  - Scopo: ultimo evento processato
  - Payload JSON:
    ```json
    { "event": "START_MOWING", "timestamp": "..." }
    ```

- `smartmower/status/transition`
  - Scopo: transizione eseguita
  - Payload JSON:
    ```json
    { "from": "IDLE", "to": "MOWING", "reason": "START_MOWING", "timestamp": "..." }
    ```

## Note di implementazione
- Gli handler MQTT sono delegati allo stato attivo tramite `StateMachine::setExternalMqttHandler()`.
- `IdleState` è event‑driven: tutta la logica di ingresso eventi avviene in `onMessage()`.
