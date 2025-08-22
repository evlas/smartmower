# Documentazione MQTT - Modulo Camera

## Panoramica
Il modulo Camera gestisce l'acquisizione di immagini da fotocamere USB o Raspberry Pi, pubblicando i frame su MQTT per l'elaborazione visiva. Il modulo supporta la configurazione di parametri di acquisizione, trasformazioni dell'immagine e fornisce informazioni dettagliate sullo stato del sistema.

## Configurazione

### File di Configurazione (robot_config.json)

```json
{
  "vision_config": {
    "camera": {
      "device": "/dev/video0",
      "width": 640,
      "height": 480,
      "fps": 30,
      "rotation": 0,
      "flip": "none",
      "height_m": 0.3,
      "intrinsics": {
        "focal_length_x": 350.0,
        "focal_length_y": 350.0,
        "principal_point_x": 320.0,
        "principal_point_y": 240.0,
        "focal_length": 600.0
      }
    }
  },
  "vision_logging": {
    "level": "info",
    "enabled": true,
    "show_windows": true,
    "camera": {
      "file": "/opt/smartmower/log/vision_camera.log",
      "data_dir": "/opt/smartmower/data/vision/camera"
    }
  },
  "mqtt": {
    "broker": "localhost",
    "port": 1883,
    "topics": {
      "camera": {
        "base": "vision/camera",
        "subtopics": {
          "data": "data",
          "status": "status",
          "commands": "commands"
        },
        "qos": 1,
        "retain": false
      }
    }
  }
}
```

## Topic MQTT

### Pubblicati

#### `{root_topic}/{base_topic}/data`
- **Frequenza**: Configurabile (default: 30 FPS)
- **Formato**: JPEG/PNG (binary)
- **QoS**: 1
- **Retain**: false
- **Metadati**: Inclusi nel topic di status

#### `{root_topic}/{base_topic}/status`
- **Frequenza**: 1Hz
- **Formato**: JSON
- **QoS**: 1
- **Retain**: false
- **Campi**:
  ```json
  {
    "online": true,
    "device": "/dev/video0",
    "width": 640,
    "height": 480,
    "fps": 29.5,
    "rotation": 0,
    "flip": "none",
    "height_m": 0.3,
    "intrinsics": {
      "focal_length_x": 350.0,
      "focal_length_y": 350.0,
      "principal_point_x": 320.0,
      "principal_point_y": 240.0
    }
  }
  ```

### Sottoscritti

#### `{root_topic}/{base_topic}/commands`
- **Formato**: JSON
- **Azione**: Comandi per il controllo della camera
- **Esempio**:
  ```json
  {
    "command": "set_parameter",
    "parameter": "fps",
    "value": 15
  }
  ```

## Parametri di Configurazione

### Camera
- `device`: Dispositivo della fotocamera (es. "/dev/video0")
- `width`: Larghezza dell'immagine in pixel
- `height`: Altezza dell'immagine in pixel
- `fps`: Frame rate desiderato
- `rotation`: Rotazione dell'immagine in gradi (0, 90, 180, 270)
- `flip`: Ribaltamento immagine ("none", "horizontal", "vertical", "both")
- `height_m`: Altezza della camera dal suolo in metri
- `intrinsics`: Parametri intrinseci della camera
  - `focal_length_x`: Lunghezza focale sull'asse x
  - `focal_length_y`: Lunghezza focale sull'asse y
  - `principal_point_x`: Punto principale sull'asse x
  - `principal_point_y`: Punto principale sull'asse y
  - `focal_length`: Lunghezza focale equivalente (opzionale)

## Logging

Il modulo supporta diversi livelli di logging configurabili:
- `debug`: Dettagliato, utile per il debug
- `info`: Informazioni generali sul funzionamento
- `warning`: Avvisi su situazioni non critiche
- `error`: Errori che non bloccano il funzionamento
- `critical`: Errori critici che impediscono il funzionamento

I log vengono salvati nel file specificato in `vision_logging.camera.file` e possono essere visualizzati anche su console.

## Gestione degli Errori

Il modulo implementa un sistema di recupero automatico che tenta di:
1. Riconnettersi alla camera in caso di errore
2. Ristabilire la connessione MQTT se persa
3. Riavviare l'acquisizione in caso di errori non recuperabili

Gli errori vengono registrati nel log con il relativo livello di gravità.

## Esempi di Uso

### Avvio del modulo
```bash
cd /opt/smartmower/bin
./vision_camera
```

### Sottoscrizione ai topic
```bash
# Sottoscrizione al topic dei dati
mosquitto_sub -t "smartmower/vision/camera/data" -v

# Sottoscrizione al topic di stato
mosquitto_sub -t "smartmower/vision/camera/status" -v
```

### Invio comandi
```bash
# Cambio del frame rate
mosquitto_pub -t "smartmower/vision/camera/commands" -m '{"command":"set_parameter","parameter":"fps","value":15}'
```
    "camera_type": "rpi|usb",
    "last_update": "2025-04-01T12:00:00Z"
  }
  ```

### Sottoscritti

#### `{base_topic}/config/set`
- **Formato**: JSON
- **Esempio**:
  ```json
  {
    "camera": {
      "fps": 10,
      "quality": 90
    }
  }
  ```

## API

### Interfaccia Base
```cpp
class CameraInterface {
public:
    virtual ~CameraInterface() = default;
    virtual bool initialize() = 0;
    virtual bool captureFrame(std::vector<uint8_t>&) = 0;
    virtual void shutdown() = 0;
};
```

## Risoluzione Problemi

### Nessun Frame Ricevuto
1. Verificare i permessi della fotocamera:
   ```bash
   ls -l /dev/video*
   ```
2. Controllare i log del servizio:
   ```bash
   journalctl -u vision_camera -f
   ```

### Latenza Elevata
1. Ridurre la risoluzione o il frame rate
2. Verificare il carico della CPU
3. Controllare la larghezza di banda di rete

### Errore MQTT
1. Verificare la connessione al broker:
   ```bash
   mosquitto_sub -h localhost -t "#" -v
   ```
2. Controllare username/password nel file di configurazione
**Comandi supportati**:
```json
// Abilita/disabilita streaming
{
  "command": "set_streaming",
  "enabled": true,
  "quality": 80,
  "resolution": "640x480"
}

// Abilita/disabilita rilevamento
{
  "command": "set_detection",
  "enabled": true,
  "threshold": 0.5
}

// Salva immagine
{
  "command": "save_image",
  "path": "/tmp/capture.jpg",
  "with_detections": true
}
```

## Formato Messaggi

### Esempio Rilevamenti
```json
{
  "timestamp": 1628671200000,
  "detections": [
    {
      "label": "person",
      "confidence": 0.87,
      "bbox": [100, 150, 200, 400],
      "distance": 2.5,
      "angle": -15.0
    },
    {
      "label": "dog",
      "confidence": 0.92,
      "bbox": [300, 200, 150, 200],
      "distance": 3.2,
      "angle": 10.0
    }
  ]
}
```

### Esempio Stato
```json
{
  "timestamp": 1628671200000,
  "streaming": true,
  "fps": 4.8,
  "resolution": "1280x720",
  "detection_enabled": true,
  "model": "yolov5s",
  "gpu_acceleration": true
}
```

## Note
- Le coordinate dei bounding box sono in pixel relativi all'immagine
- L'angolo è calcolato rispetto all'asse ottico della telecamera
- La distanza è una stima basata su ipotesi di dimensione nota degli oggetti
