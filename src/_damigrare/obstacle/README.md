# Structure from Motion (SfM) Obstacle Detection System

Sistema avanzato di rilevamento ostacoli per SmartMower basato su Structure from Motion e flusso ottico.

## Principio di Funzionamento

### Fisica del Sistema
Il sistema sfrutta il principio del **parallasse di movimento**: oggetti più vicini si muovono più velocemente nell'immagine rispetto a quelli lontani quando la telecamera si sposta. È lo stesso effetto che vedi dal finestrino di un treno: i pali vicini "sfrecciano", le montagne lontane sembrano ferme.

### Algoritmo Tecnico

1. **Cattura Frame Consecutivi**
   ```
   Frame N:    [ostacolo in posizione X1]
   Frame N+1:  [ostacolo in posizione X2] 
   Spostamento = X2 - X1 pixel
   ```

2. **Calcolo del Flusso Ottico**
   - Traccia punti caratteristici tra i frame usando Lucas-Kanade
   - Misura lo spostamento di ogni punto in pixel
   - Maggiore spostamento = oggetto più vicino

3. **Triangolazione per Calcolo Distanza**
   ```
   Distanza = (velocità_robot × baseline) / spostamento_pixel × focal_length
   ```

## Parametri del Sistema

### Precisione Attesa
- **Oggetti 1-3m**: ±15-20cm
- **Oggetti 3-5m**: ±30-50cm
- **Migliore precisione**: con movimento costante del robot

### Parametri Necessari
- **Baseline**: distanza percorsa dal robot tra i frame
- **Velocità robot**: per calcolare la baseline temporale
- **Calibrazione camera**: parametri intrinseci della telecamera
- **Timestamp**: per calcolare intervalli temporali precisi

## Vantaggi e Svantaggi

### ✅ Vantaggi
- Molto preciso per oggetti a media distanza (1-5m)
- Funziona con qualsiasi ostacolo (non serve riconoscimento)
- Robusto a diverse condizioni di luce
- No database di oggetti richiesto
- Adatto per robot in movimento costante

### ❌ Svantaggi
- Richiede movimento del robot per funzionare
- Non funziona da fermo o a velocità molto bassa
- Problemi con oggetti in movimento (persone, animali)
- Sensibile a vibrazioni della telecamera

## Installazione

### Dipendenze
```bash
# Ubuntu/Debian
sudo apt-get install libopencv-dev libmosquitto-dev libcjson-dev

# Fedora/CentOS
sudo dnf install opencv-devel mosquitto-devel libcjson-devel
```

### Compilazione
```bash
cd /home/vito/Documenti/GitHub/smartmower/vision/obstacle
make check-deps  # Verifica dipendenze
make             # Compila il sistema
```

## Configurazione

Il file `config.json` contiene tutti i parametri configurabili:

```json
{
    "mqtt": {
        "broker": "localhost",
        "port": 1883,
        "topic": "smartmower/vision/camera",
        "username": "mower",
        "password": "smart"
    },
    "robot": {
        "velocity": 0.5,           // Velocità robot in m/s
        "camera_height": 0.3,      // Altezza camera dal suolo
        "max_detection_range": 5.0 // Range massimo rilevamento
    },
    "camera": {
        "focal_length_x": 600.0,   // Lunghezza focale X
        "focal_length_y": 600.0,   // Lunghezza focale Y
        "principal_point_x": 320.0, // Punto principale X
        "principal_point_y": 240.0  // Punto principale Y
    },
    "sfm_parameters": {
        "max_corners": 100,         // Massimo numero di punti da tracciare
        "quality_level": 0.01,      // Qualità minima dei corner
        "min_distance": 10,         // Distanza minima tra corner
        "min_track_frames": 3,      // Frame minimi per validare un punto
        "min_displacement_threshold": 0.5 // Spostamento minimo in pixel
    }
}
```

## Utilizzo

### 1. Avvio del Sistema di Rilevamento
```bash
./sfm_obstacle_detector
```

### 2. Visualizzazione degli Ostacoli
```bash
# Visualizzazione completa con interfaccia grafica
python3 obstacle_viewer.py

# Solo statistiche testuali
python3 obstacle_viewer.py --stats-only

# Con parametri personalizzati
python3 obstacle_viewer.py --broker 192.168.1.100 --username custom --password secret
```

### 3. Monitoraggio MQTT
```bash
# Sottoscrizione al topic degli ostacoli
mosquitto_sub -h localhost -t "smartmower/vision/obstacles" -u mower -P smart -v
```

## Output del Sistema

### Topic MQTT: `smartmower/vision/obstacles`
```json
{
    "timestamp": "1690123456789",
    "obstacles": [
        {
            "x": 320,              // Posizione X nell'immagine
            "y": 240,              // Posizione Y nell'immagine
            "distance": 1.5,       // Distanza stimata in metri
            "velocity_x": -2.3,    // Velocità X in pixel/frame
            "velocity_y": 0.8,     // Velocità Y in pixel/frame
            "track_id": 42         // ID univoco del punto tracciato
        }
    ],
    "total_points": 87            // Totale punti tracciati
}
```

## Calibrazione della Camera

Per ottenere la massima precisione, calibra la camera usando il tool di OpenCV:

```bash
# Cattura immagini di una scacchiera
# Usa il tool di calibrazione OpenCV
# Aggiorna i parametri in config.json
```

Parametri tipici per camera USB 640x480:
- **focal_length_x/y**: 600-800 pixel
- **principal_point_x**: ~320 (metà larghezza)
- **principal_point_y**: ~240 (metà altezza)

## Ottimizzazione delle Performance

### Parametri Critici
- **max_corners**: Più punti = maggiore precisione ma più CPU
- **quality_level**: Valori più bassi = più punti ma meno affidabili
- **min_distance**: Distanza minima tra punti per evitare clustering
- **min_track_frames**: Frame minimi per considerare valido un ostacolo

### Tuning per Diverse Condizioni
- **Movimento lento**: Riduci `min_displacement_threshold`
- **Ambiente complesso**: Aumenta `quality_level`
- **Performance**: Riduci `max_corners`
- **Precisione**: Aumenta `min_track_frames`

## Debug e Troubleshooting

### Log di Debug
Il sistema salva frame di debug ogni 30 frame come `debug_frame_*.jpg` con:
- Punti tracciati colorati per distanza
- Vettori di velocità
- Etichette di distanza
- Statistiche in tempo reale

### Problemi Comuni

1. **Nessun ostacolo rilevato**
   - Verifica che il robot si stia muovendo
   - Controlla la velocità in `config.json`
   - Verifica la connessione MQTT

2. **Distanze imprecise**
   - Calibra la camera
   - Verifica la velocità del robot
   - Controlla i timestamp dei frame

3. **Troppi falsi positivi**
   - Aumenta `min_track_frames`
   - Aumenta `quality_level`
   - Riduci `max_corners`

4. **Performance basse**
   - Riduci `max_corners`
   - Aumenta `min_distance`
   - Verifica la CPU disponibile

## Integrazione con SmartMower

Il sistema si integra perfettamente con l'architettura SmartMower:

1. **Input**: Legge immagini dal topic `smartmower/vision/camera`
2. **Processing**: Elabora il flusso ottico in tempo reale
3. **Output**: Pubblica ostacoli su `smartmower/vision/obstacles`
4. **Navigation**: Il sistema di navigazione può sottoscrivere agli ostacoli

## Sviluppo Futuro

### Miglioramenti Pianificati
- [ ] Integrazione con IMU per compensare vibrazioni
- [ ] Filtro di Kalman per tracking più robusto
- [ ] Clustering degli ostacoli per ridurre rumore
- [ ] Predizione del movimento degli ostacoli
- [ ] Integrazione con sensori di distanza (LiDAR/ultrasuoni)

### Estensioni Possibili
- [ ] Riconoscimento tipo di ostacolo
- [ ] Mappa locale degli ostacoli
- [ ] Path planning dinamico
- [ ] Interfaccia web per monitoraggio remoto

## Licenza

Parte del progetto SmartMower Vision System.
Sviluppato per applicazioni robotiche autonome.

## Contatti

Per supporto tecnico o contributi al progetto, consultare la documentazione principale di SmartMower.
