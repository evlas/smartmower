# Lawn Mower Robot

Un robot tosaerba differenziale con sensori sonar, telecamera e GPS.

## Specifiche del Robot

- **Dimensioni corpo**: 60cm x 45cm x 29cm
- **Altezza da terra**: 9cm
- **Ruote posteriori**: raggio 15cm, larghezza 6cm, separate di 45cm
- **Ruota anteriore**: sfera di 4.5cm di raggio
- **Posizionamento**: ruote posteriori a 5cm dal bordo, ruota anteriore a 5cm dal bordo
- **Sensori**:
  - 3 sonar frontali per rilevamento ostacoli
  - Telecamera monoculare frontale
  - GPS al centro del corpo
  - Odometria sulle ruote differenziali
  - IMU per orientamento

## Installazione

1. Clonare il repository nel workspace ROS2
2. Compilare il progetto:
   ```bash
   cd /path/to/workspace
   colcon build --packages-select lawn_mower_robot
   ```

## Utilizzo

### 🚀 Come Utilizzare:

**Simulazione in Gazebo:**
```bash
cd /home/vito/sim_ws
source install/setup.bash
ros2 launch lawn_mower_robot gazebo.launch.py
# opzionale: specifica un world diverso
# ros2 launch lawn_mower_robot gazebo.launch.py world:=/percorso/al/tuo/world.sdf
```

**Visualizzazione in RViz:**
```bash
cd /home/vito/sim_ws
source install/setup.bash
ros2 launch lawn_mower_robot rviz.launch.py
```

**Teleop da tastiera:**
```bash
cd /home/vito/sim_ws
source install/setup.bash
ros2 launch lawn_mower_robot keyboard_teleop.launch.py
# opzionale: remap del topic
# ros2 launch lawn_mower_robot keyboard_teleop.launch.py cmd_vel_topic:=/cmd_vel
```

**Controllo con joypad (opzionale):**
```bash
cd /home/vito/sim_ws
source install/setup.bash
ros2 launch lawn_mower_robot teleop.launch.py
```

### 📡 Topics ROS2 Disponibili:

- `/cmd_vel` — Comando velocità (Twist)
- `/odom` — Odometria del robot (Odometry)
- `/joint_states` — Stati delle articolazioni
- `/imu` — Dati IMU
- `/camera/image` — Immagine telecamera (sensor_msgs/Image)
- `/camera/camera_info` — Info camera (sensor_msgs/CameraInfo)
- `/sonar/left/scan` — Sonar sinistro (sensor_msgs/LaserScan)
- `/sonar/center/scan` — Sonar centrale (sensor_msgs/LaserScan)
- `/sonar/right/scan` — Sonar destro (sensor_msgs/LaserScan)
- `/gps/fix` — Dati GPS (sensor_msgs/NavSatFix)

## Struttura del Progetto

- `urdf/`: Descrizione del robot in URDF/Xacro
  - `robot.urdf.xacro`: File principale del robot
  - `robot_core.xacro`: Geometria e cinematica del robot
  - `sensors.xacro`: Definizione dei sensori
  - `gazebo_control.xacro`: Plugin di controllo Gazebo
- `launch/`: File di launch per ROS2
- `rviz/`: Configurazione RViz
- `config/`: File di configurazione

## Troubleshooting

- **Vengono creati più robot ad ogni launch**
  - Assicurati di chiudere completamente Gazebo prima di rilanciare, oppure rimuovi l'entità esistente:
    ```bash
    ros2 service call /world/test/remove ros_gz_interfaces/srv/DeleteEntity "{name: 'lawn_mower_robot'}"
    ```
  - Nel launch è stato rimosso `-allow_renaming` per evitare spawn con suffissi (`_0`, `_1`).

- **Nessun dato su `/camera/image`**
  - Il mondo `worlds/obstacles.world` include il plugin Sensors (ogre2). Verifica i topic:
    ```bash
    ros2 topic list | grep camera
    gz topic -l | grep camera
    ```
  - La camera pubblica su `camera/image` e `camera/camera_info`, già bridgiati in `config/gz_bridge.yaml`.

- **Sonar non visibili in RViz**
  - Sono bridgiati su `/sonar/left/scan`, `/sonar/center/scan`, `/sonar/right/scan`.
  - In `rviz/lawn_mower.rviz` sono già presenti tre display LaserScan preconfigurati.

## Autore

Vito - Robotica Project
