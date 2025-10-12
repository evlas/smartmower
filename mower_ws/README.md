# Mower ROS 2 Workspace

Workspace ROS 2 per il sistema di controllo del robot tagliaerba autonomo.

## Struttura del Workspace

```
mower_ws/
├── src/                    # Pacchetti ROS 2
│   ├── pico_control_hardware/      # Hardware interface per Raspberry Pi Pico
│   ├── mower_description/          # Descrizione URDF del robot
│   ├── mower_bringup/              # File di avvio e configurazione
│   ├── battery_manager/            # Monitoraggio stato batteria
│   ├── blade_manager/              # Controllo motori lame
│   ├── relay_manager/              # Gestione relay e attuatori
│   ├── rpi_gpio/                   # Interfaccia GPIO Raspberry Pi
│   ├── safety_supervisor/          # Sistema di sicurezza e E-Stop
│   ├── state_machine/              # Macchina a stati del robot
│   ├── events_bridge/              # Bridge eventi tra componenti
│   ├── camera_ros/                 # Acquisizione camera
│   ├── ublox_gps/                  # Driver GPS Ublox
│   └── tf2_ros/                    # Static transform publisher
└── README.md               # Questo file
```

## Nodi Principali Attivi

### 1. pico_control_hardware
**Stato**: ✅ **COMPLETAMENTE FUNZIONANTE**
**Scopo**: Hardware interface completo per Raspberry Pi Pico con protocollo COBS avanzato.

#### Topic Pubblicati
- `/imu/data` (`sensor_msgs/Imu`): Dati IMU (accelerometro, giroscopio)
- `/odom` (`nav_msgs/Odometry`): Odometria e posizione del robot
- `/sonar/left/scan`, `/sonar/center/scan`, `/sonar/right/scan` (`sensor_msgs/Range`): Sensori ultrasuoni
- `/battery` (`sensor_msgs/BatteryState`): Stato batteria (tensione, corrente, livello)
- `/events/pcf8574` (`std_msgs/UInt16`): Eventi hardware
- `/blades/rpm` (`std_msgs/Float32`): RPM motori lame

#### Topic Sottoscritti
- `/blades/cmd` (`std_msgs/Float32MultiArray`): Comandi velocità lame (0-1 normalizzato)
- `/relay/cmd` (`std_msgs/Bool`): Comando relay on/off

#### Servizi
- Interfaccia hardware completa per ros2_control
- Controllo velocità ruote differenziale
- Lettura encoder ruote
- Controllo attuatori avanzato

---

### 2. battery_manager
**Stato**: ✅ **FUNZIONANTE**
**Scopo**: Monitoraggio batteria e generazione eventi di sicurezza.

#### Topic Pubblicati
- `/mower/event` (`std_msgs/String`): Eventi batteria (`LOW_BATTERY`, etc.)

---

### 3. blade_manager
**Stato**: ✅ **FUNZIONANTE**
**Scopo**: Controllo sicuro dei motori delle lame.

#### Topic Pubblicati
- `/blades/cmd` (`std_msgs/Float32MultiArray`): Comandi per lame

---

### 4. relay_manager
**Stato**: ✅ **FUNZIONANTE**
**Scopo**: Gestione relay e attuatori vari.

---

### 5. safety_supervisor
**Stato**: ✅ **FUNZIONANTE**
**Scopo**: Sistema di sicurezza centralizzato con E-Stop.

#### Topic Pubblicati
- `/safety/estop` (`std_msgs/Bool`): Stato arresto emergenza
- `/safety/status` (`diagnostic_msgs/DiagnosticArray`): Stato sistema

---

### 6. state_machine
**Stato**: ✅ **FUNZIONANTE**
**Scopo**: Macchina a stati principale del robot mower.

#### Topic Pubblicati
- `/mower/state` (`std_msgs/String`): Stato attuale del robot
- `/mower/event` (`std_msgs/String`): Eventi di stato

---

### 7. camera_ros
**Stato**: ✅ **FUNZIONANTE**
**Scopo**: Acquisizione ed elaborazione flusso video.

#### Topic Pubblicati
- `/camera/image_raw` (`sensor_msgs/Image`): Flusso video grezzo
- `/camera/camera_info` (`sensor_msgs/CameraInfo`): Informazioni camera

---

### 8. ublox_gps
**Stato**: ✅ **FUNZIONANTE**
**Scopo**: Driver GPS Ublox con configurazione avanzata.

#### Topic Pubblicati
- `/gps/fix` (`sensor_msgs/NavSatFix`): Posizione GPS
- `/gps/fix_velocity` (`geometry_msgs/TwistWithCovarianceStamped`): Velocità GPS

## Configurazione Attuale

### File di Configurazione
- `src/mower_bringup/config/ros2_controllers.yaml`: Controller ros2_control
- `src/mower_bringup/config/*.yaml`: Parametri per ogni nodo
- `src/mower_description/urdf/mower.urdf.xacro`: Modello robot completo

### Parametri Hardware
- **Dispositivo Pico**: `/dev/ttyAMA0` (UART Raspberry Pi)
- **Baud rate**: 115200
- **Update rate**: 100 Hz
- **Wheel radius**: 0.30 m
- **Wheel separation**: 0.55 m

## Avvio del Sistema

### Avvio completo:
```bash
cd /home/ubuntu/mower/mower_ws
source install/setup.bash
ros2 launch mower_bringup bringup_pico.launch.py port:=/dev/ttyAMA0 baud:=115200
```

### Verifica sistema:
```bash
ros2 node list                    # Nodi attivi
ros2 topic list                   # Topic disponibili
ros2 control list_controllers     # Controller attivi
ros2 control list_hardware_interfaces  # Interfacce hardware
```

## Stato Implementazione

### ✅ **COMPLETAMENTE FUNZIONANTE**
- Hardware interface completo per Raspberry Pi Pico
- Sistema di controllo ruote differenziale
- Acquisizione sensori (IMU, sonar, batteria, encoder)
- Sistema di sicurezza con E-Stop
- Macchina a stati del robot
- Acquisizione camera
- GPS Ublox configurato

### 🚧 **DA IMPLEMENTARE** (vedi TODO.md)
- Storage e gestione aree di taglio
- Pianificazione traiettorie di copertura
- Navigazione autonoma (Nav2 integration)
- Rilevamento ostacoli avanzato
- Missioni di taglio automatiche
- Registrazione perimetri manuale
