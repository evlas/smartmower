# SfM Obstacle Detector - Parametri di Configurazione

## 📋 Panoramica

Il sistema SfM Obstacle Detector supporta una configurazione completa tramite il file `config.json`. Tutti i parametri critici possono essere modificati senza ricompilare il codice.

## 🔧 Parametri Configurabili

### 📡 **MQTT Configuration**
```json
"mqtt": {
    "broker": "localhost",           // Indirizzo broker MQTT
    "port": 1883,                   // Porta broker MQTT
    "topic": "smartmower/vision/camera",  // Topic immagini in ingresso
    "velocity_topic": "smartmower/fusion/data",  // Topic dati fusion sensor (include velocità)
    "username": "mower",            // Username MQTT
    "password": "smart"             // Password MQTT
}
```

### 🤖 **Robot Parameters**
```json
"robot": {
    "camera_height": 0.3,           // Altezza camera dal suolo (m)
    "max_detection_range": 5.0      // Range massimo rilevamento (m)
}
```

### 📷 **Camera Calibration**
```json
"camera": {
    "focal_length_x": 350.0,        // Lunghezza focale X (pixel)
    "focal_length_y": 350.0,        // Lunghezza focale Y (pixel)
    "principal_point_x": 320.0,     // Punto principale X (pixel)
    "principal_point_y": 240.0,     // Punto principale Y (pixel)
    "width": 640,                   // Larghezza immagine
    "height": 480                   // Altezza immagine
}
```

### 🎯 **SfM Parameters**
```json
"sfm_parameters": {
    "max_corners": 150,             // Numero massimo feature da rilevare
    "quality_level": 0.005,         // Soglia qualità feature (0-1)
    "min_distance": 15,             // Distanza minima tra feature (pixel)
    "block_size": 5,                // Dimensione blocco per corner detection
    "harris_k": 0.04,               // Parametro k per Harris corner detector
    "min_track_frames": 2,          // Frame minimi per validare tracking
    "max_optical_flow_error": 100.0, // Errore massimo optical flow
    "min_displacement_threshold": 0.1 // Soglia minima displacement (pixel)
}
```

### 🚧 **Obstacle Detection Parameters**
```json
"obstacle_detection": {
    "max_distance": 5.0,            // Distanza massima ostacoli da pubblicare (m)
    "min_distance": 0.1,            // Distanza minima valida (m)
    "min_frames_tracked": 2,        // Frame minimi per validare un punto
    "max_optical_flow_error": 100.0, // Errore massimo optical flow
    "min_points_threshold": 50,     // Soglia minima punti per aggiungere nuove feature
    "displacement_threshold": 0.1,  // Soglia minima displacement per calcolo distanza
    "publish_close_obstacles_only": true  // Pubblica solo ostacoli vicini
}
```

### 🐛 **Debug Configuration**
```json
"debug": {
    "enabled": true,                // Abilita output di debug
    "log_level": "debug",           // Livello di log
    "show_windows": true            // Mostra finestre OpenCV
}
```

## 🎛️ **Tuning Raccomandato**

### Per Ambienti Interni
```json
"obstacle_detection": {
    "max_distance": 3.0,
    "min_points_threshold": 30,
    "displacement_threshold": 0.05
}
```

### Per Ambienti Esterni
```json
"obstacle_detection": {
    "max_distance": 8.0,
    "min_points_threshold": 80,
    "displacement_threshold": 0.2
}
```

### Per Debug/Test
```json
"obstacle_detection": {
    "max_distance": 10.0,
    "min_frames_tracked": 1,
    "max_optical_flow_error": 200.0
}
```

## 📊 **Output MQTT**

Il sistema pubblica su `smartmower/vision/obstacles`:

```json
{
    "obstacles": [
        {
            "x": 387.93,           // Posizione X in pixel
            "y": 267.92,           // Posizione Y in pixel  
            "distance": 4.71,      // Distanza in metri
            "velocity_x": 2.66,    // Velocità X in pixel/frame
            "velocity_y": 0.46,    // Velocità Y in pixel/frame
            "track_id": 41722      // ID univoco del punto tracciato
        }
    ],
    "total_points": 52,        // Totale punti tracciati
    "robot_velocity": 0.5      // Velocità robot in m/s
}
```

## 🚀 **Performance Tips**

1. **Ridurre scatti**: Aumentare `min_points_threshold` e `max_optical_flow_error`
2. **Maggiore precisione**: Ridurre `displacement_threshold` e aumentare `min_frames_tracked`
3. **Performance**: Ridurre `max_corners` e aumentare `min_distance`
4. **Range detection**: Modificare `max_distance` in base all'applicazione

## 🔧 **Troubleshooting**

- **Nessun ostacolo rilevato**: Ridurre `displacement_threshold` e `min_distance`
- **Troppi falsi positivi**: Aumentare `min_frames_tracked` e ridurre `max_optical_flow_error`
- **Sistema instabile**: Aumentare `min_points_threshold` e `max_optical_flow_error`
- **Prestazioni lente**: Ridurre `max_corners` e aumentare `min_distance`
