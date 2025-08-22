# SLAM – Consumo Vision Odometry via MQTT

Questo modulo SLAM consuma dati da MQTT e fonde IMU (da Pico) e GPS con la Vision Odometry (VO) pubblicata dal micro-servizio `vision/odometry`.

## Topic MQTT
- Dati Pico (bridge): `smartmower/bridge/pico/data`
  - JSON: `timestamp`, `accel`, `gyro`, `mag`, `ultrasonic`, `safety`, ...
- Vision Odometry (OBBLIGATORIO): `smartmower/vision/odometry/data`
  - JSON: `timestamp_us`, `translation {x,y,z}`, `rotation_quat {w,x,y,z}`, `confidence`.
- Pubblicazioni SLAM:
  - Pose: `mqtt.topics.slam.base` + `/` + `mqtt.topics.slam.subtopics.pose`
  - Traiettoria/Map: `mqtt.topics.slam.base` + `/` + `mqtt.topics.slam.subtopics.map`

## Configurazione (`src/config/robot_config.json`)
Sezioni rilevanti:
- `mqtt.topics.slam` per i topic di pubblicazione `pose` e `map`.
- `slam_config.subscriptions` deve contenere le seguenti chiavi:
```json
"subscriptions": {
  "vodometry": "smartmower/vision/odometry/data",
  "vobstacle": "smartmower/vision/obstacle/data",
  "gps": "smartmower/bridge/gps/data",
  "pico": "smartmower/bridge/pico/data"
}
```
Note:
- `vodometry` è OBBLIGATORIO: senza VO il nodo SLAM non parte.
- `pico` fornisce IMU/sonar/safety aggregati.
- `vobstacle` è opzionale: se presente, aggiorna la obstacle map.
- La `camera` non è supportata come fallback.

## Esecuzione e Test End-to-End
1. Avvia Vision Odometry:
   - `./src/vision/odometry/bin/vision_odometry /opt/smartmower/etc/config/robot_config.json`
2. Avvia SLAM:
   - `./src/slam/bin/slam_node /opt/smartmower/etc/config/robot_config.json`
3. Osserva:
   - VO: `mosquitto_sub -t 'smartmower/vision/odometry/data' -v`
   - Pose SLAM: `mosquitto_sub -t 'smartmower/slam/pose' -v` (o in base alla config)

## TODO Ultrasuoni
Per l'uso dei dati `ultrasonic`, vedi `docs/slam_ultrasonic_todo.md`.
