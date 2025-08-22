# Architettura Software — SmartMower

Ultimo aggiornamento: 2025-08-20 01:05:50

## Panoramica
- Linguaggi: C/C++
- Struttura header: `include/config` (lettura `/opt/smartmower/etc/config/robot_config.json`), `include/mqtt` (client/gestione MQTT)
- Build: Makefile per modulo; binari in `bin/` e install in `/opt/smartmower/bin`; oggetti in `obj/`
- Orchestrazione: solo Supervisor sotto systemd; gli altri servizi avviati/fermati dal Supervisor
- Config: `src/config/robot_config.json` (con profili batteria in `src/config/battery_profiles.json`)

## Schema architetturale (alto livello)
```
                     +---------------------------+
                     |       Web Admin (C++)     |
                     |  - HTTP server + UI       |
 User <-> Browser ---+  - /api/config, /api/...  +---- MQTT (libmosquitto)
                     +-------------+-------------+
                                   |
                                   v
+------------------+     +---------+---------+      +------------------+
|  Supervisor      |     | State Machine     |      |  Safety Service  |
|  (systemd unit)  |     |  (FSM C++)        |      |  (C++)           |
| - start/stop     |     | - stati INIT/...  |      | - estop/tilt...  |
| - health/watch   |     | - eventi MQTT     |      | - battery state  |
+-----+------------+     +----+---------+----+      +---------+--------+
      |                         |         |                    |
      |                         |         |                    |
      |                         |         |                    |
      |                  +------+         +------+             |
      |                  | Navigation: Costmap  |             |
      |                  |  (C++)               |             |
      |                  +--+--------+----------+             |
      |                     |        |                        |
      |                     |        |                        |
      |               Global Planner Local Planner            |
      |                    (C++)          (C++)               |
      |                         \        /                    |
      |                          v      v                    |
      |                       Controller (PID/MPPI)          |
      |                              |                        |
      |                              v                        |
      |                       Bridge Attuatori (Pico)  <------+  Safety gating
      |                              |
      |                              v
      |                        HW (motori/relè/blade)
      |
      +---> Bridge GPS / Vision Obstacle / SLAM (altri servizi) --- MQTT bus
```

Bus di comunicazione: MQTT (libmosquitto). Ogni servizio pubblica/sottoscrive topic definiti in `robot_config.json` (radice `mqtt.root_topic`, es. `smartmower/...`).

## Moduli e responsabilità
- Supervisor
  - Avvia/ferma/riavvia i servizi; unico gestito da systemd.
  - Health/Watchdog su topic `.../status` dei servizi.
  - Comandi MQTT per start/stop dei servizi (es. `smartmower/supervisor/start`).
  - Policy di restart e shutdown configurabili in `robot_config.json`.
  - Heartbeat su topic `.../status` per monitorare lo stato dei servizi.
- State Machine (`src/state_machine/`)
  - Stati: `INIT`, `IDLE`, `MOWING`, `PAUSED`, `DOCKING`, `UNDOCKING`, `CHARGING`, `MANUAL_CONTROL`, `EMERGENCY_STOP`, `ERROR`.
  - Transizioni data‑driven da `src/config/robot_config.json` (`allowed_events`, `transitions`).
  - Pubblica stato/eventi su `smartmower/status/*`.
- Safety Service (`src/safety/`)
  - E‑Stop, tilt/lift, batteria (fallback multipli), geofence; publish `smartmower/safety/*`.
- Navigation
  - CostmapNode: fonde `slam/map`, obstacle, sonar → `nav/costmap`.
  - Global Planner: `nav/global_path`.
  - Local Planner (DWA/TEB): `nav/cmd_vel`.
  - Controller (PID/MPPI): tracking `nav/cmd_vel`.
- Bridge Attuatori Pico (`src/bridge/pico/`)
  - Subscribe `.../cmd_vel_safe`/`cmd_vel`, `blade_cmd`; comandi verso Pico `.../bridge/pico/commands/{motors,system}`.
- Bridge GPS (`src/bridge/gps/`)
  - Normalizza dati GPS → topic SLAM/navigation.
- Vision/Obstacle (`src/vision/`)
  - Rilevamento ostacoli da camera; publish `.../obstacle`.
- Web Admin (`web/`)
  - UI e API per configurazione e controllo manuale.

## Flussi principali
- Missione e controllo manuale
  - Web Admin → `smartmower/mission/state` → `IdleState` / `ManualState`.
  - Manuale: comandi unificati `smartmower/state_machine/commands` → `ManualState` → bridge Pico.
- Navigazione autonoma
  - `nav/costmap` → planner globale → planner locale → `nav/cmd_vel` → Safety Supervisor → `nav/cmd_vel_safe` → Bridge Pico → HW.
- Safety
  - `safety/state` ed `estop` influenzano FSM (es. `EMERGENCY_STOP`, `BATTERY_LOW`).

## Build & Deploy
- Ogni servizio ha Makefile dedicato (bin in `bin/`, obj in `obj/`).
- `sudo make install` installa i binari in `/opt/smartmower/bin` e copia i file di servizio (solo Supervisor in systemd).
- Header condivisi sotto `include/config` e `include/mqtt`.

## Stato attuale vs. mancante (gap)
- Completato (principale)
  - FSM: `INIT`, `IDLE`, `MANUAL_CONTROL`, `EMERGENCY_STOP`, `PAUSED`, `ERROR` completi.
  - FSM base presenti: `MOWING`, `DOCKING`, `UNDOCKING`, `CHARGING` (core pronto, da estendere come sotto).
  - Web Admin base + manuale; Safety core; CostmapNode in corso.
- Mancante / Da sviluppare
  - Navigation
    - Global Planner — ID: `nav-global`
    - Local Planner — ID: `nav-local`
    - Controller PID/MPPI — ID: `ctrl-pid`
    - Test end‑to‑end costmap/planner — ID: `costmap-tests`
  - Safety Supervisor (gating `cmd_vel` → `cmd_vel_safe`) — ID: `safe-supervisor`
  - Bridge Attuatori Pico (completo E2E) — ID: `act-bridge`
  - Mission Manager (waypoints, no‑go) — ID: `mis-manager`
  - Persistenza/Logging mappe/telemetria — ID: `log-persist`
  - Ops: target `make install` per tutti i moduli — ID: `ops-make`
  - Testing: simulatori MQTT (cmd_vel, blade, obstacle, sonar) — ID: `test-sim`
  - Web: install/service e test runtime — ID: `web-install`, `web-test`
  - FSM estensioni
    - MOWING: obstacle→PAUSE, battery_low→DOCKING, telemetria blade — ID: `sm-mowing-*`
    - DOCKING/UNDOCKING: gestione timeout e completion — ID: `sm-docking-*`, `sm-undocking-*`
    - CHARGING: battery_full→IDLE, hysteresis — ID: `sm-charging-*`

## Riferimenti
- Configurazione: `src/config/robot_config.json`
- Stato TODO: `docs/PROJECT_TODO.md` (sezioni Backlog e FSM)
