# Tests MQTT per SLAM

Questa cartella contiene piccoli tool in C++ per simulare i flussi MQTT attesi dal nodo SLAM:

- `mqtt/imu_publisher.cpp`: invia pacchetti binari IMU conformi a `pico::SensorData` sul topic `smartmower/pico/data`.
- `mqtt/gps_publisher.cpp`: invia un JSON GPS con `lat/lon/alt/timestamp` su `smartmower/gps/data`.
- `mqtt/cam_publisher.cpp`: invia un singolo frame JPEG su `smartmower/vision/camera/data` (richiede OpenCV).

I binari vengono creati in `tests/bin/` e l'installazione li copia in `/opt/smartmower/bin/tests`.

## Requisiti
- `libmosquitto` (client): su Debian/Ubuntu `sudo apt-get install libmosquitto-dev`.
- (Opzionale per Camera) OpenCV 4: `sudo apt-get install libopencv-dev`.
- Broker MQTT in esecuzione (es. `mosquitto` su localhost:1883).

## Build

```bash
make -C tests
```

Se OpenCV è rilevato, verrà compilato anche `cam_pub`. Altrimenti verrà mostrato un messaggio informativo.

## Installazione

```bash
sudo make -C tests install
```

Installa i binari in `/opt/smartmower/bin/tests`.

## Esecuzione

- IMU (100 pacchetti ~100Hz):

```bash
./tests/bin/imu_pub               # usa localhost:1883 e topic di default
# oppure con argomenti: broker port topic
./tests/bin/imu_pub 192.168.1.10 1883 smartmower/pico/data
```

- GPS (singolo messaggio):

```bash
./tests/bin/gps_pub
# oppure
./tests/bin/gps_pub localhost 1883 smartmower/gps/data
```

- Camera (invia un JPEG):

```bash
# Assicurati di avere un'immagine in tests/data/sample_camera.jpg
./tests/bin/cam_pub localhost 1883 smartmower/vision/camera/data tests/data/sample_camera.jpg
```

## Dati di esempio
- `tests/data/sample_gps.json`: payload JSON conforme a quanto atteso dal nodo SLAM.
- `tests/data/sample_camera.jpg`: NON incluso. Aggiungi un qualsiasi JPEG con nome `sample_camera.jpg` per provare il publisher camera.

## Note
- I publisher sono pensati per essere semplici: nessuna gestione QoS avanzata, retain, o riconnessioni.
- Verifica i topic nel tuo `src/config/robot_config.json` e adegua i comandi se necessario.
