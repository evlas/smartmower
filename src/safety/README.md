# Safety Supervisor

Modulo che ascolta i dati grezzi dal bridge Pico (topic `smartmower/bridge/pico/data`) e pubblica:
- `smartmower/safety/estop` con `{ "pressed": true|false }`
- `smartmower/safety/state` con `{ "battery_low": true|false }`

La logica è minimale: usa `battery_pct` se presente nel payload Pico e confronta con `tuning.battery_thresholds.low_threshold`. In assenza di `battery_pct`, non modifica lo stato batteria (estendibile con profili batteria).

## Build

```
make -C src/safety
```

## Install

```
sudo make -C src/safety install PREFIX=/opt/smartmower
```

## Disinstallazione

```
sudo make -C src/safety uninstall PREFIX=/opt/smartmower
```

## Configurazione
- File: `/opt/smartmower/etc/config/robot_config.json` (fallback `src/config/robot_config.json`)
- Chiavi usate:
  - `mqtt.{broker,port,username,password,root_topic}`
  - `mqtt.topics.{pico,safety}` per costruire i topic completi
  - `tuning.battery_thresholds.low_threshold`

## Note
- L'avvio/stop del processo è gestito dal Supervisor (unico modulo che usa systemd).
- Estendibile per integrare altri segnali di sicurezza (tilt, rain, bumper) aggregandoli in `safety/state`.
