# 🤖 Smart Mower - Robot Tagliaerba Autonomo

<div align="center">

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-5-red.svg)](https://www.raspberrypi.com/)
[![MicroPython](https://img.shields.io/badge/MicroPython-Pico-yellow.svg)](https://micropython.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

**Robot tagliaerba autonomo intelligente con navigazione GPS, sensori avanzati e controllo remoto.**

[📖 Documentazione](#-documentazione) • [🚀 Installazione](#-installazione) • [⚙️ Configurazione](#️-configurazione) • [🎯 Utilizzo](#-utilizzo)

</div>

---

## 📋 Sommario

**Smart Mower** è un sistema robotico completo per il taglio automatico dell'erba, composto da:

- **🖥️ Raspberry Pi 5** - Computer principale con ROS 2 (navigazione e controllo)
- **🔌 Raspberry Pi Pico** - Controller hardware per sensori e attuatori (IMU, odometria, safety)
- **📡 GPS uBlox F10N** - Navigazione e localizzazione (RTK opzionale futuro)
- **📷 Camera Raspicam 1** - Visione artificiale monoculare (telecamera profondità futura)
- **🔋 Sistema batteria** - Monitoraggio e gestione energetica
- **🛡️ Sistema di sicurezza** - E-Stop e sensori di sicurezza

Il robot è in grado di mappare aree di lavoro, pianificare percorsi di taglio ottimali e operare in completa autonomia.

## 🏗️ Architettura del Sistema

```
┌─────────────────────────────────────────────────────────────────┐
│                    🌐 Smart Mower System                        │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────┐  │
│  │  Raspberry  │  │  Raspberry  │  │    GPS     │  │ Camera  │  │
│  │  Pi 5 (ROS2)│◄►│  Pi Pico    │  │  uBlox     │  │Raspicam │  │
│  │   Navigation│  │  (Sensors)  │  │   F10N     │  │   v1    │  │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────┘  │
│        │                │                 │              │       │
│        ▼                ▼                 ▼              ▼       │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────┐  │
│  │  Navigation │  │    IMU      │  │  Odometry   │  │ Safety  │  │
│  │  Controller │  │   Data      │  │   Data      │  │ System  │  │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────┘  │
│        │                       ▲                       │       │
│        └───────────────────────┼───────────────────────┘       │
│                                │                               │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────┐  │
│  │   Mower     │  │   Safety    │  │  Actuators  │  │ Blades  │  │
│  │ Controller  │  │ Supervisor  │  │  Controller │  │Manager  │  │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

**Flussi di navigazione:**
- **Camera (Raspicam)** → RTAB-Map → Navigation Controller
- **GPS (uBlox F10N)** → Navigation Controller  
- **IMU (da Pico)** → Navigation Controller
- **Odometria (da Pico)** → Navigation Controller

**Flussi di controllo:**
- **Navigation Controller** → Mower Controller → Pico (attuatori)
- **Safety Supervisor** ↔ Pico (sensori di sicurezza)
- **Blade Manager** → Pico (controllo lame)

## 📁 Struttura del Repository

```
mower/
├── 📁 hardware/                    # Documentazione hardware
│   ├── SCH_smartmower_1-Mother Board_2025-10-12.png  # Schema elettrico
│   └── ProPrj_smartmower_2025-07-31_08-52-16_2025-10-12.epro  # Progetto KiCad
│
├── 📁 mower_ws/                    # Workspace ROS 2
│   ├── src/                        # Pacchetti sorgente
│   │   ├── pico_control_hardware/  # Hardware interface per Pico
│   │   ├── mower_bringup/          # Launcher e configurazione
│   │   ├── safety_supervisor/      # Sistema di sicurezza
│   │   ├── state_machine/          # Macchina a stati
│   │   ├── battery_manager/        # Monitoraggio batteria
│   │   ├── blade_manager/          # Controllo motori lame
│   │   ├── relay_manager/          # Gestione relay
│   │   ├── rpi_gpio/               # Interfaccia GPIO Raspberry Pi
│   │   ├── events_bridge/          # Bridge eventi
│   │   └── mower_description/      # Descrizione URDF del robot
│   └── README.md                   # Documentazione ROS 2 dettagliata
│
├── 📁 pico_micropython_ws/         # Firmware Raspberry Pi Pico
│   └── firmware/                   # Codice MicroPython per sensori/attuatori
│       ├── main.py                 # Firmware principale Pico
│       ├── app/                    # Moduli applicazione
│       └── README.md               # Documentazione firmware
│
├── 📄 mower.code-workspace         # Configurazione VS Code
├── 📄 uart_test.py                 # Script di test comunicazione UART
└── 📄 README.md                    # Questo file
```

## 🎯 Caratteristiche

### ✅ **Funzionalità Implementate**
- **Comunicazione UART avanzata** con protocollo COBS tra Raspberry Pi e Pico
- **Controllo motori differenziale** con encoder per navigazione precisa
- **Sistema di sicurezza completo** con E-Stop e monitoring batteria
- **Acquisizione multi-sensore**: IMU, sonar (3x), batteria, eventi hardware
- **Macchina a stati** per gestione operativa del robot
- **Controllo lame e relay** per operazioni di taglio sicure
- **Monitoraggio batteria** con soglie configurabili
- **Sistema eventi** per diagnostica e sicurezza

### 🚧 **In Sviluppo**
- **Mappatura aree** con registrazione perimetri manuale
- **RTAB-Map monoculare** per mappatura 3D con camera Raspicam
- **Algoritmi di copertura** per ottimizzazione percorsi di taglio
- **Navigazione autonoma** con GPS uBlox F10N
- **Missioni di taglio complete** con gestione errori e recovery

## 🛠️ Requisiti Hardware

### Componenti Principali
| Componente | Modello | Funzione |
|------------|---------|----------|
| **Computer principale** | Raspberry Pi 5 (8GB) | ROS 2, navigazione, AI, RTAB-Map |
| **Controller hardware** | Raspberry Pi Pico | Sensori (IMU, odometria) e attuatori |
| **GPS** | uBlox F10N | Posizionamento preciso per navigazione |
| **Camera** | Raspicam Module v1 | Visione monoculare per RTAB-Map |
| **Motori** | 2x DC motor + encoder | Movimento differenziale |
| **Lame** | 2x motori brushless | Sistema di taglio |
| **Batteria** | Li-Ion 12.6V | Alimentazione principale |
| **Sensori** | IMU MPU6050, 3x sonar | Orientamento e ostacoli (via Pico) |

### Pinout Raspberry Pi 5
```
Raspberry Pi 5 GPIO (40-pin header)
┌─────────────────────────────────────┐
│  3.3V  5V  GND  14  15  18  GND  23 │
│   2   3   4   17  27  22  25  24    │
│   7  GND  8   31  GND  26  GND  19  │
│  10   9  11  GND   5   6  12  13   │
│  16  26  20  GND  21  19  16  26   │
└─────────────────────────────────────┘
```

### Collegamenti Principali
- **UART**: Pico ↔ Raspberry Pi (TX/RX/GND)
- **I2C**: IMU, display OLED
- **GPIO**: Encoder ruote, finecorsa, relay
- **PWM**: Controllo motori (ESC)
- **ADC**: Monitoraggio batteria

## 🚀 Installazione

### Prerequisiti
```bash
# Sistema operativo
sudo apt update && sudo apt upgrade
sudo apt install python3-pip git cmake build-essential

# ROS 2 Humble
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-humble-desktop

# MicroPython per Pico
sudo apt install python3-venv
pip3 install thonny  # IDE per Pico
```

### Build del Progetto
```bash
# Clona il repository
git clone <repository-url>
cd mower

# Build workspace ROS 2
cd mower_ws
colcon build --symlink-install

# Installa dipendenze Python
pip3 install -r requirements.txt  # Se presente

# Setup ambiente ROS 2
echo 'source ~/mower/mower_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### Firmware Raspberry Pi Pico
```bash
# Installa MicroPython su Pico
# 1. Tieni premuto BOOTSEL su Pico e collega via USB
# 2. Copia firmware MicroPython nella nuova unità (RPI-RP2)
# 3. Scollega e ricollega Pico

# Carica il codice del progetto
cd pico_micropython_ws/firmware
# Usa Thonny o ampy per caricare main.py e app/ su Pico
```

## ⚙️ Configurazione

### File di Configurazione Principali
```yaml
# mower_ws/src/mower_bringup/config/ros2_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

diff_drive_controller:
  ros__parameters:
    wheel_separation: 0.56  # Distanza tra ruote (m)
    wheel_radius: 0.15      # Raggio ruote (m)
    # ... altri parametri
```

### Parametri Hardware
```bash
# Dispositivo UART per Pico
export MOWER_UART_DEVICE="/dev/ttyAMA0"
export MOWER_UART_BAUD="115200"

# Configurazione batteria
export MOWER_BATTERY_MAX_V="12.6"  # 3S LiPo full
export MOWER_BATTERY_MIN_V="9.0"   # 3S LiPo critical
```

## 🎮 Utilizzo

### Avvio Sistema Base
```bash
# Avvio completo del sistema
ros2 launch mower_bringup bringup_pico.launch.py

# Verifica nodi attivi
ros2 node list
ros2 topic list
ros2 service list

# Monitoraggio sistema
ros2 topic echo /battery          # Stato batteria
ros2 topic echo /imu/data         # Dati IMU
ros2 topic echo /sonar/center/scan # Sensori distanza
```

### Controllo Manuale
```bash
# Movimento manuale
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5}"  # Avanti
ros2 topic pub /cmd_vel geometry_msgs/Twist "angular: {z: 1.0}" # Rotazione

# Controllo lame (manuale - usa con cautela!)
ros2 topic pub /blades/cmd std_msgs/Float32MultiArray "data: [0.5, 0.5]"
```

### Monitoraggio e Debug
```bash
# Visualizzazione TF
ros2 run tf2_tools view_frames.py

# Rviz2 per visualizzazione 3D (se configurato)
rviz2 -d mower.rviz

# Logs del sistema
ros2 bag record /imu/data /odom /battery /sonar/center/scan
```

## 🔧 Sviluppo

### Ambiente di Sviluppo
```bash
# VS Code con estensioni ROS
code mower.code-workspace

# Estensioni consigliate:
# - ROS (ms-iot.vscode-ros)
# - Python (ms-python.python)
# - C/C++ (ms-vscode.cpptools)
```

### Test del Sistema
```bash
# Test comunicazione UART
python3 ../uart_test.py

# Test componenti hardware
ros2 run mower_bringup hardware_test.py

# Test RTAB-Map (quando implementata)
ros2 launch rtabmap_ros rtabmap.launch.py \
  rtabmap_args:="--delete_db_on_start" \
  rgb_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info \
  approx_sync:=false

# Test algoritmi copertura (quando implementata)
ros2 run mower_coverage coverage_planner_test.py
```

### Debug e Troubleshooting
```bash
# Monitoraggio seriale Pico
sudo minicom -D /dev/ttyAMA0 -b 115200

# Debug ROS 2
export RCUTILS_LOGGING_LEVEL=DEBUG
ros2 launch mower_bringup bringup_pico.launch.py
```

## 📚 Documentazione Dettagliata

### Sistema Base
- **ROS 2 Humble** - Framework robotico
- **Raspberry Pi OS** - Sistema operativo
- **MicroPython** - Firmware per Raspberry Pi Pico

### Pacchetti ROS 2
- **rclcpp** - Client C++ ROS 2
- **sensor_msgs** - Messaggi sensori
- **geometry_msgs** - Messaggi geometria
- **diagnostic_msgs** - Messaggi diagnostica
- **tf2_ros** - Trasformazioni
- **cv_bridge** - Bridge OpenCV
- **camera_info_manager** - Gestione informazioni camera
- **hardware_interface** - Interfacce hardware ros2_control

### Pacchetti Futuri Previsti
- **rtabmap_ros** - Mappatura 3D monoculare
- **nav2** - Navigazione autonoma
- **mower_coverage** - Algoritmi ottimizzazione percorsi
- **robot_localization** - Fusione sensori (EKF)

- **[📖 Documentazione ROS 2](mower_ws/README.md)** - Guida completa al workspace ROS 2
- **[🔧 Hardware](hardware/)** - Schema elettrico e documentazione hardware
- **[⚙️ Firmware Pico](pico_micropython_ws/firmware/README.md)** - Protocollo e documentazione firmware
- **[🧪 Test](uart_test.py)** - Script di test e validazione

## 🤝 Contribuire

1. Fork il repository
2. Crea un branch per la tua feature (`git checkout -b feature/amazing-feature`)
3. Commit le tue modifiche (`git commit -m 'Add amazing feature'`)
4. Push su GitHub (`git push origin feature/amazing-feature`)
5. Apri una Pull Request

### Linee Guida per il Codice
- Segui lo stile ROS 2 C++ per i nodi
- Commenta il codice in inglese
- Aggiungi test per nuove funzionalità
- Aggiorna la documentazione

## 📄 Licenza

Questo progetto è distribuito sotto licenza **MIT**. Vedi il file [LICENSE](LICENSE) per i dettagli.

## 🙏 Ringraziamenti

- **ROS 2 Community** per l'eccellente framework robotico
- **Raspberry Pi Foundation** per l'hardware accessibile
- **MicroPython Team** per il firmware embedded

## 📞 Supporto

Per domande, problemi o suggerimenti:

- 📧 **Email**: [tuo-email@esempio.com]
- 💬 **Issues**: [GitHub Issues](https://github.com/tuo-username/smart-mower/issues)
- 📖 **Wiki**: [Documentazione completa](https://github.com/tuo-username/smart-mower/wiki)

---

<div align="center">

**⭐ Se trovi utile questo progetto, lascia una stella!**

[![Stars](https://img.shields.io/github/stars/tuo-username/smart-mower?style=social)](https://github.com/tuo-username/smart-mower)
[![Forks](https://img.shields.io/github/forks/tuo-username/smart-mower?style=social)](https://github.com/tuo-username/smart-mower)

*Costruito con ❤️ per l'automazione intelligente del giardinaggio*

</div>
