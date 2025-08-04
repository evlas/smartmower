# Perimeter Detector

Questo modulo rileva il perimetro del muretto (alto più di 35cm) utilizzando l'elaborazione delle immagini e pubblica i risultati su MQTT.

## Requisiti

- OpenCV 4.x
- libmosquitto
- libcjson
- CMake 3.10+

## Compilazione

```bash
mkdir build
cd build
cmake ..
make
```

## Esecuzione

1. Assicurati che il broker MQTT sia in esecuzione:
   ```bash
   mosquitto -v
   ```

2. Esegui il rilevatore di perimetro:
   ```bash
   ./perimeter_detector
   ```

## Configurazione

Il programma può essere configurato modificando direttamente il codice sorgente. I parametri principali sono:

- `min_wall_height`: altezza minima del muretto in cm (default: 35cm)
- `focal_length`: lunghezza focale della telecamera in pixel
- `camera_height`: altezza della telecamera dal suolo in cm

## Output MQTT

I risultati vengono pubblicati sul topic `smartmower/vision/perimeter` in formato JSON:

```json
{
  "walls": [
    {
      "x": 100,
      "y": 200,
      "width": 150,
      "height": 80,
      "estimated_height_cm": 40.5
    }
  ],
  "timestamp": 1627000000000
}
```
