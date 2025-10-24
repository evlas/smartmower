# 🤖 Smart Mower - Robot Tagliaerba Autonomo

<div align="center">

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-5-red.svg)](https://www.raspberrypi.com/)
[![C++](https://img.shields.io/badge/C%2B%2B-Pico%20SDK-blue.svg)](https://github.com/raspberrypi/pico-sdk)
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
- **Camera (Raspicam)** → RTAB-Map → Navigation Controller (Nav2)
- **GPS (uBlox F10N)** → Navigation Controller (Nav2)
- **IMU (da Pico)** → Navigation Controller (Nav2)
- **Odometria (da Pico)** → Navigation Controller (Nav2)

**Flussi di controllo:**
- **Navigation Controller (Nav2)** → Mower Controller → Pico (attuatori)
- **Safety Supervisor** ↔ Pico (sensori di sicurezza)
- **Blade Manager** → Pico (controllo lame)

## 📁 Struttura del Repository

```
mower/
├── 📁 hardware/                    # Documentazione hardware
│   ├── SCH_smartmower_1-Mother Board_2025-10-12.png  # Schema elettrico
│   └── ProPrj_smartmower_2025-07-31_08-52-16_2025-10-12.epro  # Progetto EasyEDA Pro
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
├── 📁 pico_cpp_ws/                # Firmware Raspberry Pi Pico (C++)
│   └── firmware/                   # Codice C++ per sensori/attuatori
│       ├── src/                    # Sorgenti (.cpp/.hpp)
│       ├── CMakeLists.txt          # Configurazione build
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
- **Navigazione autonoma** con Nav2 e RTAB-Map per mappatura 3D
- **Modalità ibrida**: SLAM per mappatura e localizzazione con mappe esistenti
- **Twist Mux intelligente** per gestione sicura dei comandi di velocità per stato

### 🚧 **In Sviluppo**
- **Mappatura aree** con registrazione perimetri manuale
- **Algoritmi di copertura** per ottimizzazione percorsi di taglio
- **Missioni di taglio complete** con gestione errori e recovery
- **Ottimizzazioni RTAB-Map** per ambienti esterni complessi
- **Integrazione GPS avanzata** con RTK per precisione centimetrica

## 🛠️ Requisiti Hardware

### Componenti Principali
| Componente | Modello | Funzione |
|------------|---------|----------|
| **Computer principale** | Raspberry Pi 5 (8GB) | ROS 2, navigazione, AI, RTAB-Map |
| **Controller hardware** | Raspberry Pi Pico | Sensori (IMU, odometria) e attuatori |
| **GPS** | uBlox F10N | Posizionamento preciso per navigazione |
| **Camera** | Raspicam Module v1 | Visione monoculare per RTAB-Map |
| **Motori** | 2x motori brushless + encoder | Movimento differenziale |
| **Lame** | 2x motori brushless | Sistema di taglio |
| **Batteria** | Li-Ion 6S | Alimentazione principale |
| **Sensori** | IMU MPU6050, 3x sonar | Orientamento e ostacoli (via Pico) |

## 🚀 Installazione

### Prerequisiti
```bash
# Sistema operativo
sudo apt update && sudo apt upgrade
sudo apt install python3-pip git cmake build-essential

# ROS 2 Jazzy
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-jazzy-desktop

# Navigazione autonoma (Nav2) e mappatura (RTAB-Map)
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-rtabmap-ros

# Pico SDK per firmware C/C++ del Raspberry Pi Pico
# Segui la sezione "Firmware Raspberry Pi Pico" qui sotto per l'installazione e la configurazione complete del Pico SDK.
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

⚠️ **IMPORTANTE**: Non utilizzare `pico_micropython_ws` in quanto MicroPython non supporta le frequenze I2C richieste dai sensori. Utilizzare invece `pico_cpp_ws` con il firmware C++.

#### Reperire e Configurare Pico SDK

```bash
# 1. Installa dipendenze per la compilazione
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential

# 2. Clona il Pico SDK
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init

# 3. Configura ambiente
echo 'export PICO_SDK_PATH="$HOME/pico-sdk"' >> ~/.bashrc
echo 'export PICO_TOOLCHAIN_PATH="/usr"' >> ~/.bashrc
source ~/.bashrc

# 4. Verifica installazione
cmake --version  # Dovrebbe mostrare versione >= 3.13
arm-none-eabi-gcc --version  # Dovrebbe mostrare toolchain ARM
```

#### Compilare il Firmware C++

```bash
# 1. Entra nella directory del firmware C++
cd pico_cpp_ws/firmware

# 2. Crea directory build
mkdir build
cd build

# 3. Configura CMake con Pico SDK
cmake -DCMAKE_BUILD_TYPE=Release ..

# 4. Compila il firmware
make -j$(nproc)

# 5. Il firmware compilato sarà in build/src/firmware.elf e build/src/firmware.uf2
```

#### Caricare il Firmware su Pico

```bash
# 1. Tieni premuto il pulsante BOOTSEL su Pico e collega via USB
# 2. Pico apparirà come unità RPI-RP2
# 3. Copia il file .uf2 nella nuova unità
cp build/src/firmware.uf2 /media/$USER/RPI-RP2/

# 4. Pico si riavvierà automaticamente con il nuovo firmware
# 5. Verifica l'output via script di sniffing seriale (protocollo Pico)
python3 /home/ubuntu/mower/pico_cpp_ws/utils/serial_imu_sniffer.py --pico --port /dev/ttyAMA0 --baud 230400
# In alternativa, se usi la porta USB CDC del Pico:
# python3 /home/ubuntu/mower/pico_cpp_ws/utils/serial_imu_sniffer.py --pico --port /dev/ttyACM0 --baud 115200
```

#### Debug e Monitoraggio

```bash
# Monitoraggio output firmware
python3 /home/ubuntu/mower/pico_cpp_ws/utils/serial_imu_sniffer.py --pico --port /dev/ttyAMA0 --baud 230400

# Debug con OpenOCD (se disponibile)
# 1. Collega Pico in modalità debug (SWD)
# 2. openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg
# 3. gdb-multiarch build/src/firmware.elf
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

### Parametri Nav2 + RTAB-Map
```bash
# Modalità navigazione
export MOWER_LOCALIZATION_MODE="True"  # True: localizzazione, False: SLAM

# Percorso mappa (per modalità localizzazione)
export MOWER_MAP_FILE="/home/ubuntu/mower/mower_ws/maps/map.yaml"

# Configurazione RTAB-Map
export MOWER_RTABMAP_DB="/home/ubuntu/mower/mower_ws/maps/rtabmap.db"
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

### Avvio Nav2 + RTAB-Map (Sistema Completo)
```bash
# Modalità localizzazione con mappa esistente (default)
ros2 launch mower_bringup bringup_pico_rtab.launch.py

# Modalità SLAM per mappatura autonoma
ros2 launch mower_bringup bringup_pico_rtab.launch.py localization:=False

# Verifica nodi Nav2 e RTAB-Map attivi
ros2 node list | grep -E "(nav2|rtabmap|twist_mux|stop_velocity)"

# Monitoraggio navigazione
ros2 topic echo /odom                    # Odometria
ros2 topic echo /map                     # Mappa (se in localizzazione)
ros2 topic echo /rtabmap/map             # Mappa RTAB-Map (se in SLAM)
ros2 topic echo /twist_mux/select        # Selezione mux corrente
```

### RTAB-Map Standalone (Mappatura Manuale)
```bash
# Avvia RTAB-Map per mappatura indipendente
ros2 launch mower_bringup rtabmap.launch.py

# Visualizza la mappa in costruzione
ros2 run rtabmap_viz rtabmap_viz

# Salva mappa RTAB-Map
ros2 service call /rtabmap/save_map rtabmap_ros/SaveMap
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
ros2 bag record /imu/data /odom /battery /sonar/center/scan /camera/image_raw

# Debug nodi specifici
ros2 run rqt_graph rqt_graph  # Visualizza grafo nodi
ros2 run rqt_console rqt_console  # Monitora log

# Monitoraggio Twist Mux
ros2 topic echo /twist_mux/select     # Stato corrente selezionato
ros2 topic echo /cmd_vel_mux         # Comando velocità finale
ros2 topic list | grep cmd_vel       # Tutti i topic cmd_vel disponibili
```

### Twist Mux - Gestione Sicura dei Comandi di Velocità

Il sistema utilizza **Twist Mux** per garantire che solo i comandi appropriati per lo stato corrente possano controllare il robot:

#### **Topic cmd_vel per Stato:**
- `/mower/cmd_vel/undocking` - Comandi per UNDOCKING
- `/mower/cmd_vel/mowing` - Comandi per MOWING
- `/mower/cmd_vel/docking` - Comandi per DOCKING
- `/mower/cmd_vel/manual` - Controllo manuale (alta priorità)
- `/mower/cmd_vel/stop` - Velocità zero per stati di stop

#### **Come Funziona:**
1. **State Machine** pubblica lo stato corrente su `/mower/state`
2. **TwistMuxSelector** riceve lo stato e pubblica la selezione su `/twist_mux/select`
3. **StopVelocityPublisher** pubblica continuamente velocità zero su `/mower/cmd_vel/stop`
4. **Twist Mux** riceve la selezione e abilita solo l'input cmd_vel corrispondente
5. L'output va direttamente al controller: `/diff_drive_controller/cmd_vel`

#### **Gestione Stati di Stop:**
Quando il robot è in **IDLE**, **EMERGENCY_STOP**, **CHARGING**, **ERROR** o **PAUSED**:
- Viene selezionato l'input `stop_cmd_vel` (priorità 0)
- Il `StopVelocityPublisher` garantisce velocità zero costanti
- **Nessun movimento** è possibile in questi stati

#### **Priorità del Sistema:**
- **MANUAL_CONTROL** (100): Massima priorità per controllo umano
- **DOCKING** (40): Alta priorità per operazioni critiche
- **MOWING** (30): Priorità normale per taglio
- **UNDOCKING** (20): Priorità media per uscita
- **IDLE/EMERGENCY_STOP/CHARGING/ERROR/PAUSED** (0): Stati di stop - nessun movimento

#### **Vantaggi:**
- ✅ **Sicurezza**: Solo comandi appropriati per stato attivo
- ✅ **Gestione conflitti**: Priorità chiare tra stati
- ✅ **Debug facilitato**: Topic separati per ogni stato
- ✅ **Estensibilità**: Facile aggiungere nuovi stati

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

# Test algoritmi copertura (quando implementata)
ros2 run mower_coverage coverage_planner_test.py
```

### Debug e Troubleshooting
```bash
# Monitoraggio seriale Pico
python3 /home/ubuntu/mower/pico_cpp_ws/utils/serial_imu_sniffer.py --pico --port /dev/ttyAMA0 --baud 230400

# Debug ROS 2
export RCUTILS_LOGGING_LEVEL=DEBUG
ros2 launch mower_bringup bringup_pico.launch.py
```

## 📚 Documentazione Dettagliata

### Sistema Base
- **ROS 2 Jazzy** - Framework robotico
- **Raspberry Pi OS** - Sistema operativo
- **Pico SDK** - Framework C/C++ per Raspberry Pi Pico

### Pacchetti ROS 2
- **rclcpp** - Client C++ ROS 2
- **sensor_msgs** - Messaggi sensori
- **geometry_msgs** - Messaggi geometria
- **diagnostic_msgs** - Messaggi diagnostica
- **tf2_ros** - **[📖 Documentazione ROS 2](mower_ws/README.md)** - Guida completa al workspace ROS 2
- **[🔧 Hardware](hardware/)** - Schema elettrico e documentazione hardware
- **[⚙️ Firmware Pico](pico_cpp_ws/firmware/README.md)** - Protocollo e documentazione firmware C++
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
- **Raspberry Pi Foundation** per l'hardware accessibile e il Pico SDK
- **Pico SDK Team** per l'eccellente toolchain di sviluppo C/C++

## 📞 Supporto

Per domande, problemi o suggerimenti:

- 📧 **Email**: [vito.ammirata[at]gmail.com]

---

<div align="center">

**⭐ Se trovi utile questo progetto, lascia una stella!**

[![Stars](https://img.shields.io/github/stars/tuo-username/smart-mower?style=social)](https://github.com/tuo-username/smart-mower)
[![Forks](https://img.shields.io/github/forks/tuo-username/smart-mower?style=social)](https://github.com/tuo-username/smart-mower)

*Costruito con ❤️ per l'automazione intelligente del giardinaggio*

</div>
