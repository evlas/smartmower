# SmartMower State Machine (skeleton)

Questo modulo implementa uno scheletro della macchina a stati del robot, ispirato alla vecchia
implementazione in `smarmower_old/_damigrare/state_machine/`.

Stati previsti (placeholders):
- INIT
- IDLE
- MOWING
- PAUSED
- DOCKING
- UNDOCKING
- CHARGING
- MANUAL_CONTROL
- EMERGENCY_STOP
- ERROR

Per ora la logica reale non è implementata: il ciclo principale stampa il nome dello stato
corrente e gestisce transizioni simulate su input futuro (TODO) o timeout.

## Struttura
- include/config/config_manager.h: lettura configurazione da `/opt/smartmower/etc/config/robot_config.json` (fallback a `src/config/robot_config.json`).
- include/mqtt/mqtt_client.h: stub client MQTT (connect/publish/subscribe no-op).
- include/state_machine/state_machine.h: definizione `StateMachine` e `StateId`.
- src/config/config_manager.cpp: implementazione lettura JSON con nlohmann::json.
- src/mqtt/mqtt_client.cpp: implementazione stub.
- src/state_machine.cpp: core macchina a stati (scheletro).
- src/main.cpp: entry-point.
- Makefile: build in `obj/`, bin in `bin/`, install in `/opt/smartmower/bin`.
  L'avvio/stop del processo è gestito dal Supervisor (unico modulo che usa systemd).

## Build

```bash
make
```

Output:
- Binario: `bin/state_machine`

## Installazione

```bash
sudo make install PREFIX=/opt/smartmower
```

Installa:
- Binario in `/opt/smartmower/bin/state_machine`

## Note
- Dipendenze: richiede `nlohmann/json` (header-only). Assicurarsi che l'include path sia disponibile.
- La logica reale degli stati verrà implementata successivamente.
