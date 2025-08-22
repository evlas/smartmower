# Vision Odometry Micro-servizio

Questo micro-servizio sottoscrive i frame della camera da MQTT, calcola l'odometria visiva (R, t, confidenza) e pubblica un JSON su MQTT per ridurre la banda.

## Topic MQTT
- Input frame: `smartmower/vision/camera/data`
- Output odometria: `smartmower/vision/odometry/data`
- Output stato: `smartmower/vision/odometry/status`

Payload pubblicato su `vision/odometry/data`:
```json
{
  "timestamp_us": 1234567890,
  "translation": {"x": 0.01, "y": -0.02, "z": 0.00},
  "rotation_quat": {"w": 0.99, "x": 0.01, "y": 0.00, "z": 0.02},
  "confidence": 0.86
}
```

## Build
```bash
make -C src/vision/odometry
```
Il binario viene generato in `src/vision/odometry/bin/vision_odometry`.

## Installazione
```bash
sudo make -C src/vision/odometry install
```

## Configurazione
Il servizio legge la configurazione da `/opt/smartmower/etc/config/robot_config.json` (o un path passato come argomento).

Sezioni rilevanti:
- `mqtt.root_topic`, `mqtt.broker`, `mqtt.port`, `mqtt.username`, `mqtt.password`
- `mqtt.topics.camera` e `mqtt.topics.odometry`
- `vision_config.camera.intrinsics` (opzionale: fx, fy, cx, cy)

### Parametri Vision Odometry (`vision_config.odometry`)
Controllano tracking, stima moto e pubblicazione. Esempio:
```json
"vision_config": {
  "odometry": {
    "features": { "max_corners": 500, "quality_level": 0.01, "min_distance": 8, "block_size": 5 },
    "optical_flow": { "win_size": 21, "max_level": 3, "term_eps": 0.03 },
    "ransac": { "prob": 0.999, "threshold_px": 1.0, "min_inliers": 10 },
    "tracking": { "min_features_refill": 200, "refill_corners": 500 },
    "publish": { "every_n_frames": 1, "min_confidence": 0.2 }
  }
}
```
- **features**: parametri di `goodFeaturesToTrack`.
  - `max_corners`: numero massimo di corner da estrarre.
  - `quality_level`: soglia qualità Shi-Tomasi (0..1).
  - `min_distance`: distanza minima tra corner.
  - `block_size`: dimensione finestra per la misura.
- **optical_flow**: parametri di `calcOpticalFlowPyrLK`.
  - `win_size`: lato finestra LK (pixel).
  - `max_level`: livelli piramidali.
  - `term_eps`: epsilon per criterio di terminazione (con 30 iterazioni max).
- **ransac**: robustezza nella stima dell'Essential matrix.
  - `prob`: probabilità RANSAC.
  - `threshold_px`: soglia di errore (pixel).
  - `min_inliers`: inlier minimi per accettare la stima.
- **tracking**: mantenimento delle feature.
  - `min_features_refill`: se i match validi scendono sotto questa soglia, ricarica feature.
  - `refill_corners`: numero di corner da ricaricare.
- **publish**: controllo pubblicazione VO.
  - `every_n_frames`: pubblica ogni N frame validi.
  - `min_confidence`: confidenza minima (0..1) per pubblicare.

## Esecuzione manuale
```bash
./src/vision/odometry/bin/vision_odometry /opt/smartmower/etc/config/robot_config.json
```

## Note
- L'avvio/stop del processo è gestito dal Supervisor (unico modulo che usa systemd).
- Dipendenze: OpenCV 4, libmosquitto, nlohmann/json (header-only).
- Il servizio usa feature tracking (LK optical flow) e `findEssentialMat/recoverPose`.
