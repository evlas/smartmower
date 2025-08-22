# Project TODO — SmartMower

Ultimo aggiornamento: 2025-08-20 23:21:09

## Indice
- Panoramica stato
- Backlog prioritizzato (Now/Next/Later)
- Attività dettagliate per area
- State Machine (dettagli e classificazione stati)
- Web Admin
- Supervisor
- Note

## Legenda
- Stati: [pending] da fare, [in_progress] in corso, [completed] completato
- Priorità: [high], [medium], [low]

## Stato sintetico
- In corso: definizione architettura runtime e mappa topic; sviluppo vision/obstacle; test odometria ruote Pico in SLAM
- Completato: integrazione SLAM per obstacle topic e sonar parsing; VO obbligatoria; config SLAM allineata; subscribe/parse `pico/status` e fusione odometria in EKF

## Backlog prioritizzato
- Now (alta priorità, immediato)
  - [in_progress][high] Definire architettura runtime e mappa dei topic MQTT end-to-end — id: plan1
  - [in_progress][high] Allineare `src/config/robot_config.json` con nuove sezioni — id: cfg1
  - [in_progress][high] Navigation: implementare CostmapNode e pubblicazione `nav/costmap` — id: nav-costmap
  - [pending][high] Safety Supervisor: gating `nav/cmd_vel` e pubblicazione `nav/cmd_vel_safe` — id: safe-supervisor
- Next (poi)
  - [pending][high] Global Planner — id: nav-global
  - [pending][high] Local Planner — id: nav-local
  - [pending][high] Controller (PID/MPPI) — id: ctrl-pid
  - [pending][high] Bridge attuatori Pico — id: act-bridge
- Later (successivo)
  - [pending][medium] Mission Manager — id: mis-manager
  - [pending][medium] Persistenza/Logging — id: log-persist
  - [pending][medium] Ops: Makefile install per ogni modulo — id: ops-make

## Attività

- [completed][high] Refactor: spostare costmap in navigation/map come Map Server unico (bin: map_server) — id: refactor_navigation_map
- [completed][high] Makefile: aggiornato build root per usare `src/navigation/map` e rimosso target costmap — id: update_makefiles_map_server
- [completed][medium] Config: aggiunto topic `mqtt.topics.nav` con subtopic `map` (pubblicazione multilayer) — id: update_config_topics_nav_map
- [pending][medium] Pulizia: rimuovere directory `src/navigation/costmap/` dopo migrazione completa — id: remove_costmap_dir

- [in_progress][medium] Creare e testare micro-servizio vision/obstacle (rilevamento ostacoli via camera) — id: ob1
- [pending][high] Testare vision/obstacle con optical flow: usare `tests/bin/cam_pub` e verificare MQTT output — id: ob2
- [pending][medium] Tuning parametri SFM: `min_points_threshold`, `displacement_threshold`, `max_optical_flow_error` — id: ob3
- [pending][low] Aggiornare documentazione se emergono modifiche post-test (README e `src/config/robot_config.json`) — id: ob4

- [completed][high] Integrare in SLAM: sottoscrizione topic obstacle e parsing sonar in `onDataMessage` — id: sl1
- [completed][high] SLAM: rimosso fallback camera; VO via MQTT obbligatoria (errore se `slam_config.subscriptions.vodometry` assente) — id: sl-vo-required
- [completed][medium] Config: aggiunta sezione `mqtt.topics.slam` (inizialmente `pose`, `map`, `status`) — id: cfg-slam-topics
- [completed][medium] Config/Map Server: rimosso subtopic `slam/map` (ora `mqtt.topics.slam.subtopics` contiene solo `pose` e `status`); nel `map_server` la subscribe a `slam/map` è opzionale e disattiva se non configurata — id: remove-slam-map
- [completed][medium] Config: `slam_config.subscriptions` rinominato a `{vodometry,vobstacle,gps,pico}` e codice/README aggiornati — id: cfg-slam-subs-rename

- [completed][high] SLAM: subscribe `pico/status` e dispatch `onPicoStatusMessage` — id: sl-pico-sub
- [completed][high] SLAM: parsing JSON encoder/RPM/speed e calcolo dL/dR, d_center, d_theta — id: sl-pico-parse
- [completed][high] SensorFusion: aggiunto metodo `processPicoOdometry` che richiama `EKF::updatePicoOdometry` — id: sl-pico-sf
- [completed][medium] EKF: confermata/usi `updatePicoOdometry(po, dt)` per aggiornare posizione/velocità — id: sl-pico-ekf
- [pending][high] Test end-to-end: pubblicare sample su `smartmower/bridge/pico/status`, verificare traiettoria/pose su `slam/pose` e consistenza con VO — id: sl-pico-tests
 - [completed][high] SLAM: build end-to-end OK dopo introduzione `wheel_track` e parsing `system.hardware` — id: sl-build-20250820
 - [completed][high] Allineare firmware e bridge sui comandi sistema 0x03/0x04/0x05 (SET_RELAY/CALIBRATE/RESET_ENCODERS): enum e mapping MQTT aggiornati — id: pico-cmd-0x03-align

- [in_progress][high] Definire architettura runtime e mappa dei topic MQTT end-to-end — id: plan1
- [in_progress][high] Allineare `src/config/robot_config.json` con topic e nuove sezioni (navigation, controller, actuators, safety, mission) — id: cfg1
  - [completed][high] Creato file minimo `src/config/robot_config.json` con sezioni pico/gps/camera/state_machine/supervisor — id: cfg1-min
  - [completed][high] Aggiunta lista globale `state_machine.events` e stub `allowed_events` per tutti gli stati in `mqtt.topics.state_machine.states` — id: cfg1-sm-events
  - [completed][high] Aggiunte `transitions` per-stato allineate agli `allowed_events` — id: cfg1-sm-trans
  - [completed][high] Generati profili batteria 12V/24V/36V in `src/config/battery_profiles.json` a partire da `src/config/robot_config.json.save` — id: cfg1-batt-profiles
  - [completed][medium] Safety: QoS/retain parametrizzabili per publish (`smartmower/safety/state`, `smartmower/safety/estop`) e QoS subscribe Pico — id: safety-qos-retain
  - [completed][high] Config Hardware: usare `system.hardware.dimensions.wheel_track` per la carreggiata; rimosso `wheel_base` — id: cfg-hw-wheeltrack
  - [completed][high] SLAM Config: lettura `wheel_diameter`, `wheel_track`, `motors_encoder_pulses_per_rev*motors_gear_ratio` in `loadConfig()` — id: sl-hw-params
  - [completed][high] Safety Geofence: JSON `polygon` come array di coppie `[[lat,lon], ...]` (non oggetti `{lat,lon}`) — id: safety-geofence-format
  - [completed][high] Safety: lettura campi annidati da Pico (`safety.emergency`, `power.bus_voltage`), gating su `emergency_stop_enabled`, pubblicazione `voltage_v` — id: safety-nested-fields
  - [completed][medium] Documentazione: aggiornati `docs/mqtt/safety_mqtt.md` (schema estop booleano e nuovo state) e rimando in `docs/mqtt/pico_mqtt.md` — id: docs-safety-mqtt
  - [completed][high] Safety: tilt:true implica E‑Stop (latching) e invio comando a Pico con `reason: "tilt"`; fix parsing geofence (lettura array polygon) — id: safety-tilt-geofence
  - [completed][high] Safety: fallback batteria (tensione/percentuale) da `power.*`, `voltage_v`, `battery.*`, `battery_pct`; persistenza e pubblicazione coerente in `safety/state` — id: safety-battery-fallbacks
  - [completed][high] Documentazione: allineato `docs/mqtt/pico_mqtt.md` all'implementazione (topic base, `/data`, `/status`, `/commands/*`) — id: docs-pico-mqtt-update
  - [completed][high] Pico MQTT: schema JSON `/data` documentato (campi e unità) — id: docs-pico-json-data
  - [completed][high] Pico MQTT: schema JSON `/status` documentato — id: docs-pico-json-status
  - [completed][high] Pico MQTT: comandi `/commands/{motors,system}` documentati con payload/esempi — id: docs-pico-commands
  - [completed][low] Pico MQTT: collegati `mqtt.root_topic` e `mqtt.topics.pico.base` alla composizione dei topic — id: docs-pico-config-keys
  - [completed][high] Pico MQTT: aggiornata documentazione comandi sistema: `set_relay`(0x03), `calibrate`(0x04), `reset_encoders`(0x05) — id: docs-pico-syscmd-0x03
  - [completed][high] Firmware Pico: implementata routine di calibrazione IMU/MAG per comando 0x04 (flag system_flags bit1) — id: firmware-calibration-impl
  - [completed][medium] SLAM: aggiunto `slam_config.subscriptions.pico_status` e subscribe attiva in `mqtt_slam_node.cpp` — id: sl-pico-status-sub
  - [completed][medium] SLAM Config: aggiornato `mapping.width_meters/height_meters` a 100.0 — id: cfg-slam-mapping-100
  - [completed][medium] SLAM Config: aggiunte `initial_position` e `initial_orientation` (quat identità) — id: cfg-slam-initial
  - [completed][medium] SLAM Config: aggiunta sezione `obstacle_detection` (min_distance/max_distance/filter_window_size) — id: cfg-slam-obstacle-det
  - [completed][medium] SLAM Config: aggiornato `sensor_fusion.sensor_weights.magnetometer` a 0.3 e aggiunta `mag_calibration{offset,scale,align_matrix}` — id: cfg-slam-magcal

- [in_progress][high] Navigation: implementare CostmapNode (fusiona `slam/map`, ostacoli, sonar) e pubblica `nav/costmap` — id: nav-costmap
  - [completed][medium] CostmapNode: aggiunto override path config via ENV/CLI — id: costmap-cfg-override
  - [completed][high] CostmapNode: creati unit systemd e README con istruzioni install/run — id: costmap-systemd-readme
  - [pending][medium] CostmapNode: integrare layer statico da `slam/map` (occupancy) nella griglia — id: costmap-static-layer
  - [pending][medium] CostmapNode: test end-to-end con mosquitto_pub/sub e script test dedicato — id: costmap-tests
  - [completed][medium] CostmapNode: pubblicazione status 1 Hz su `nav/costmap/status` — id: costmap-status
- [pending][high] Navigation: implementare Global Planner (A*/D*) che pubblica `nav/global_path` — id: nav-global
- [pending][high] Navigation: implementare Local Planner (DWA/TEB) che pubblica `nav/cmd_vel` — id: nav-local

- [pending][high] Controller: implementare controller PID/MPPI per tracking di `nav/cmd_vel` con feedback SLAM — id: ctrl-pid
- [pending][high] Bridge attuatori: modulo per invio comandi a Pico (sottoscrive `nav/cmd_vel_safe`/`cmd_vel` e `blade_cmd`) — id: act-bridge

- [completed][medium] Tests: aggiunto `tests/mqtt/pico_status_publisher.cpp` e target `tests/bin/pico_status_pub` per smoke test SLAM via encoder — id: tests-pico-status

- [completed][medium] SLAM: eliminata tripla inizializzazione dell'EKF all'avvio (inizializzazione unica; `setState()` usato in `setInitialPosition`/`setInitialOrientation`) — id: slam-ekf-init

- [completed][medium] Tests: aggiunti publisher per tutti i topic consumati da SLAM: `vo_pub` (vodometry), `vobstacle_pub` (vision obstacle), `pico_data_pub` (IMU/mag/ultrasonic/safety) con targets in `tests/Makefile` e install — id: tests-slam-inputs

- [pending][high] Safety Supervisor: gating di `nav/cmd_vel` basato su bumper/sonar/obstacle/E-Stop; pubblica `nav/cmd_vel_safe` e `safety/state` — id: safe-supervisor
  - [pending][low] Testing: script end-to-end per Safety (tilt/lift/estop, batteria, geofence) in `tests/mqtt/` — id: safety-e2e-tests

- [pending][medium] Mission Manager: generazione waypoint (lawnmower/spirale), aree no-go e bordi; pubblica `mission/goal` e `mission/state` — id: mis-manager

- [pending][low] Persistenza: logger di mappe/telemetria e gestione storage mappe — id: log-persist

- [pending][medium] Ops: Makefile con target install per ogni modulo in `/opt/smartmower/bin` e struttura `include/{config,mqtt}` — id: ops-make
- [completed][high] Ops: rimozione unit systemd dai moduli non‑Supervisor; systemd usato solo dal Supervisor — id: ops-systemd
 - [completed][high] Ops: Fix `Makefile` root (ricette con TAB), `make clean`/`make` ok — id: ops-make-root-fix
 - [completed][medium] Build: rimossi warning di compilazione in `state_machine/src/mqtt/mqtt_client.cpp`, `safety/src/mqtt/mqtt_client.cpp`, `web/src/web_admin.cpp` — id: build-warnings-removed-20250819

## Web Admin
- [completed][high] Implementare servizio web admin con server `libmicrohttpd` e UI statica — id: web1
- [completed][medium] Build: Makefile con bin in `web/bin/`, obj in `web/obj/`, link a `supervisor/src/mqtt/mqtt_client.cpp` — id: web-build
- [completed][high] UI config: schemi specifici per tab (MQTT/System/SLAM/Navigation/Vision) e renderer a path annidati — id: web-tab-schemas
- [completed][high] UI config: aggiunto tab Hardware con selezione `hardware.battery_type` dai `battery_profiles` e modifica dimensioni — id: web-tab-hardware
 - [completed][low] UI: rinominato tab "Supervisor" in "System" e allineati ID form/listeners — id: web-tab-rename-system
 - [completed][medium] UI Hardware: mostra dettagli profilo batteria sotto il selettore; campi modificabili: `capacity_ah`, `charging.*`; altri in sola lettura — id: web-battery-profile-editable
- [completed][medium] UI config: aggiunto tab Pico (dopo Hardware) con form su `pico_config` e `pico_logging`, pulsanti Ricarica/Salva — id: web-tab-pico
- [completed][medium] UI config: aggiunto tab GPS (dopo Pico) con form su `gps_config` e `gps_logging`, pulsanti Ricarica/Salva — id: web-tab-gps

- [completed][high] UI: modularizzazione per tab (index leggero, loader `app.js`, utilità condivise `js/core.js`, contenuti per tab in `static/tabs/*.html|*.js`) — id: web-ui-modularization

- [completed][high] Backend: endpoint `POST /api/supervisor/cmd` per inoltrare comandi a `<root>/supervisor/cmd` — id: web-backend-supervisor-endpoint
- [completed][high] UI: nuova tab "Supervisor" con controlli Start/Stop/Restart servizi — id: web-ui-supervisor-tab
 - [completed][high] UI: Supervisor — elenco moduli con toggle "Autostart at boot" persistito in `system.supervisor.autostart` via `/api/config` — id: web-ui-supervisor-autostart

### Area Utente e Autenticazione Admin
- [completed][high] Area Utente iniziale: pagina `/` con mappa placeholder, stato macchina, pulsanti Start/Pausa/Stop/Docking, batteria% — id: web-user-area
- [completed][high] Backend: routing statici separati (`/user_static/*` per utente, `/admin_static/*` + `/admin` per admin; fallback `/static/*` in sviluppo) — id: web-routing-split
- [completed][high] Backend: API utente `GET /api/state`, `POST /api/state/cmd`, `GET /api/battery` — id: web-user-api
- [completed][medium] Sicurezza: protezione area admin con PIN a 6 cifre tramite cookie `admin_pin` e `POST /api/admin/login` — id: web-admin-pin
- [completed][high] Rinomina fisica: `web/static` → `web/admin_static`, aggiornati path in HTML/JS, routing server e Makefile install — id: web-admin-static-rename
- [completed][high] Backend: rimozione warning compilazione (inizializzazione `headers` in `HttpResponse`) — id: web-warnings-headers
- [completed][high] Backend Manuale: `/api/manual/enter` pubblica `{"state":"manual"}` su `smartmower/mission/state` (coerente con `IdleState`) — id: web-backend-manual-enter
- [completed][medium] UI Manuale: migliorata UX (hold-to-drive, scorciatoie WASD, rate limit 10Hz, preset velocità) — id: web-ui-manual-ux
- [pending][medium] UI Admin: form per cambio PIN (`system.admin_pin`) con salvataggio via `POST /api/config` — id: web-admin-pin-ui

## State Machine
- [completed][high] Creato scheletro macchina a stati in `src/state_machine/` con Makefile, bin/obj, include/config e include/mqtt, servizio systemd e README — id: sm-state-machine-skeleton
- [completed][high] Pubblicazione MQTT ricca (JSON) con `current`, `previous`, `last_event`, `last_reason`, `time_in_state_sec`, `tick_count`, `timestamp` su topic `smartmower/status/state|event|transition` — id: sm-state-machine-rich-mqtt
- [completed][medium] Migrazione: `InitChecker` incorporato in `InitState` e rimosso dal build (handler MQTT esterno per INIT) — id: sm-initchecker-migrated
- [completed][high] Migrazione: logica `IDLE` spostata in `IdleState` con subscribe e handler MQTT per eventi `START_MOWING`/`BATTERY_LOW`/`MANUAL_CONTROL`/`EMERGENCY_STOP` — id: sm-idlestate-migrated
- [completed][high] Stato `MANUAL_CONTROL`: implementato `ManualState` con subscribe a `manual.cmd_vel_topic`, `manual.blade_cmd_topic`, `manual.exit_topic` e passthrough su `control.cmd_vel_topic`/`control.blade_cmd_topic`; uscita via `exit|resume|true` — id: sm-manual-state
 - [completed][high] ManualState: su ingresso accende relè Pico e su uscita lo spegne; inoltra `cmd_vel`/`blade` al bridge Pico su `.../commands/{motors,system}` — id: sm-manual-relay-forward
- [completed][high] Migrazione data-driven: `allowed_events`/`transitions` letti da `src/config/robot_config.json` e applicati a runtime; `init_timeout` gestito con `EVENT_TIMEOUT`→transizione da tabella — id: sm-data-driven
- [completed][high] Test automatico FSM: aggiunto `tests/bin/fsm_tester` che esercita transizioni via MQTT leggendo broker/porta/utente/password da `src/config/robot_config.json` — id: sm-fsm-autotest
- [completed][medium] MANUAL_CONTROL: unificato schema comandi su `smartmower/state_machine/commands` con payload JSON `{ "manual":{...} }` (cmd_vel/blade/exit), mantenendo retrocompatibilità con topic `smartmower/manual/*` — id: sm-manual-unified-cmd
- [completed][high] Web Admin: aggiornati endpoint `/api/manual/*` per inviare i comandi manuali sul topic unificato della FSM con payload JSON — id: web-admin-manual-fsm-cmd
- [completed][medium] Documentazione: aggiunta sezione “Comandi Manuali (topic unificato)” in `docs/mqtt/state_machine_mqtt.md` con esempi JSON — id: docs-manual-json-cmds
- [completed][medium] INIT: quando Pico si connette inviare comando relay OFF su `bridge/pico/commands/system` — id: sm-init-pico-relay-off
- [completed][high] Stato EMERGENCY_STOP: onEnter spegne il relè, pubblica `safety/estop=true` (retained) se necessario; accetta comando JSON `{ "recover": true }` → evento `EVENT_EMERGENCY_RECOVER` — id: sm-emergency-stop-state
- [completed][low] StateMachine: creati header `.h` per tutti gli stati (MOWING, PAUSED, DOCKING, UNDOCKING, CHARGING, ERROR) — id: sm-states-headers
- [completed][high] Policy relè: ON entrando in UNDOCKING/MOWING/DOCKING, OFF entrando in tutti gli altri stati; implementata pubblicazione su `.../bridge/pico/commands/system` — id: sm-relay-policy
- [completed][high] Stato PAUSED: gestione comando `resume`→`EVENT_RESUME` (rientro in MOWING) e forwarding eventi da `safety.state` (`BATTERY_LOW`, `ERROR_OCCURRED`) oltre a `EMERGENCY_STOP` — id: sm-paused-resume
- [pending][high] Installazione: `sudo make install` + `systemctl enable --now smartmower-webadmin.service` — id: web-install
- [pending][high] Test runtime: verificare `GET /api/config`, `POST /api/config` e serving statici `/` — id: web-test

### FSM - Stati: classificazione attuale

Stati completi:
- INIT
- IDLE
- MANUAL_CONTROL
- EMERGENCY_STOP
- PAUSED
- ERROR

Stati base da estendere (core-basis):
- MOWING
- DOCKING
- UNDOCKING
- CHARGING

#### Sottotask per stati base

- [pending][high] MOWING: pausa automatica su obstacle (`obstacle.detected==true`) → `EVENT_PAUSE`; resume manuale da `PAUSED` — id: sm-mowing-obstacle-pause
- [pending][high] MOWING: battery low da `safety.state.battery_low` → `EVENT_BATTERY_LOW` → transizione a `DOCKING` — id: sm-mowing-batt-low
- [pending][medium] MOWING: telemetria blade on/off e corrente lama (se disponibile) su topic status — id: sm-mowing-blade-telemetry
- [pending][medium] MOWING: rate limiting e debouncing comandi da `mission/state` — id: sm-mowing-cmd-debounce

- [pending][high] DOCKING: gestione timeout `docking_timeout_ms` con `EVENT_TIMEOUT` → `STATE_ERROR` — id: sm-docking-timeout
- [pending][high] DOCKING: completamento docking via `mission/state {"state":"docked"}` → `EVENT_DOCKING_COMPLETE` → `CHARGING` — id: sm-docking-complete
- [pending][medium] DOCKING: policy relè ON in ingresso, OFF se `ERROR`/`EMERGENCY_STOP` — id: sm-docking-relay-policy

- [pending][high] UNDOCKING: gestione timeout `undocking_timeout_ms` con `EVENT_TIMEOUT` → `STATE_ERROR` — id: sm-undocking-timeout
- [pending][medium] UNDOCKING: conferma uscita dal dock (`mission/state {"state":"undocked"}`) → `EVENT_UNDOCKING_COMPLETE` → `MOWING` — id: sm-undocking-complete

- [pending][high] CHARGING: transizione a `IDLE` su `battery_full==true` → `EVENT_BATTERY_FULL` — id: sm-charging-batt-full
- [pending][medium] CHARGING: pubblicazione stato carica/tempo stimato su topic status — id: sm-charging-status-publish
- [pending][low] CHARGING: hysteresis su percentuale per evitare toggle tra `CHARGING`/`IDLE` — id: sm-charging-hysteresis

## Supervisor
- [in_progress][high] Supervisor Node: orchestratore dei servizi (start/stop/restart, health via status) — id: sup1
  - [in_progress][high] API MQTT config minimal (get/set/apply senza versioning) — id: sup-config-api
  - [pending][high] Mappa impatto servizio←sezioni config e restart selettivo — id: sup-impact-map
  - [pending][medium] Health: uso dei topic `status` dei servizi per watchdog (timeout→restart) — id: sup-health-status
  - [completed][medium] Deployment: usare systemd solo per il Supervisor — id: sup3
  - [completed][high] Sostituire stub MQTT con client reale libmosquitto — id: sup-mqtt

- [pending][medium] Testing: simulatori MQTT per `cmd_vel`, `blade_cmd`, ostacoli, sonar e KPI di navigazione/safety — id: test-sim

 - [pending][high] State Machine: progettare e implementare stati/transizioni/timeouts (`init`, `idle`, `mowing`, `charging`, `emergency`) — id: sm1
 - [pending][high] State Machine: integrazione MQTT (subscribe sensori/safety/mission, publish `mission/state`, gating `blade_cmd`) — id: sm2
 - [pending][medium] State Machine: creare skeleton C++ `state_machine_node` con `include/{config,mqtt}`, Makefile, systemd, README — id: sm3
 - [pending][medium] State Machine: test end-to-end delle transizioni (init_complete, start_mowing, low_battery, obstacle, docked) — id: sm4

## Note
- I topic e le nuove sezioni di config devono essere allineati in `src/config/robot_config.json`.
- Ogni micro-servizio deve avere Makefile (bin in `bin/`, obj in `obj/`), install in `/opt/smartmower/bin`, header in `include/config` e `include/mqtt`.
- Creare file systemd e README dedicati per ciascun servizio.
