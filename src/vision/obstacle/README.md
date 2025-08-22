# Vision Obstacle Detection

Micro-servizio che sottoscrive i frame camera via MQTT, rileva ostacoli con un'approssimazione leggera basata su contorni/edge density e pubblica eventi su MQTT.

Da ora integra anche un semplice filtro di movimento (SFM) basato su optical flow (LK) per validare la presenza di ostacoli tramite numero di tracce con spostamento significativo.

## Topic MQTT
- Input frame: `mqtt.topics.camera.base + / + subtopics.data`
- Output: `mqtt.topics.obstacle.base + / + subtopics.data`
- Status: `mqtt.topics.obstacle.base + / + subtopics.status`

## Payload output (esempio)
```json
{
  "timestamp_us": 1234567890,
  "obstacle_detected": true,
  "confidence": 0.62,
  "area_ratio": 0.09,
  "image_size": {"w": 640, "h": 480},
  "bbox": {"x": 120, "y": 200, "w": 180, "h": 140}
}
```

## Configurazione
Il servizio legge `/opt/smartmower/etc/config/robot_config.json`.

Chiavi rilevanti:
- `mqtt.root_topic`, `mqtt.broker`, `mqtt.port`, `mqtt.username`, `mqtt.password`
- `mqtt.topics.camera` e `mqtt.topics.obstacle`
- `obstacle_detection.filter_window_size` (smoothing area)
- `vision_config.detection.obstacle_detection.parameters` (immagine):
  - `min_area_ratio` (default 0.05)
  - `canny_low` (50), `canny_high` (150), `dilate_iter` (1)
- `vision_config.detection.obstacle_detection.sfm_parameters` (optical flow):
  - `max_corners` (150), `quality_level` (0.005), `min_distance` (15), `block_size` (5), `harris_k` (0.04)
  - `min_track_frames` (2)
  - `max_optical_flow_error` (100.0)
  - `min_points_threshold` (70)
  - `displacement_threshold` (2.0)

## Build
```bash
make -C src/vision/obstacle
```

## Installazione
```bash
sudo make -C src/vision/obstacle install
```

## Note
- L'avvio/stop del processo è gestito dal Supervisor (unico modulo che usa systemd).
- Algoritmo: contorni su Canny + soglia area (smoothed) + validazione SFM con optical flow LK (conteggio tracce con spostamento minimo).
- Parametri regolabili via config.
- Dipendenze: OpenCV 4, libmosquitto, nlohmann/json.
