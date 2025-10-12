# mower_state_machine

Nodo ROS 2 C++ che implementa la macchina a stati del robot tosaerba.

## Stati
- IDLE
- UNDOCKING
- MOWING
- DOCKING
- CHARGING
- EMERGENCY_STOP
- MANUAL_CONTROL
- ERROR
- PAUSED

## Eventi e Transizioni
- Da IDLE:
  - UNDOCK -> UNDOCKING
  - MANUAL_ON -> MANUAL_CONTROL
  - CHARGE -> CHARGING
  - ERROR -> ERROR

- Da UNDOCKING:
  - UNDOCK_DONE -> MOWING
  - PAUSE -> PAUSED
  - ERROR -> ERROR

- Da MOWING:
  - PAUSE -> PAUSED
  - DOCK -> DOCKING
  - LOW_BATTERY -> DOCKING
  - ERROR -> ERROR
  
- Da DOCKING:
  - DOCKED -> CHARGING
  - ERROR -> ERROR

- Da CHARGING:
  - CHARGED -> IDLE
  - ERROR -> ERROR

- Da EMERGENCY_STOP:
  - RESET_ESTOP -> IDLE

- Da MANUAL_CONTROL:
  - MANUAL_OFF -> IDLE
  - ERROR -> ERROR

- Da ERROR:
  - CLEAR_ERROR -> IDLE

- Da PAUSED:
  - RESUME -> previous state (UNDOCKING, MOWING, DOCKING)
  - ERROR -> ERROR

Eventi globali:
- ESTOP -> EMERGENCY_STOP (da qualunque stato)
- TILT TRUE -> EMERGENCY_STOP (da qualunque stato, via `/safety/tilt`)
- LIFT TRUE -> EMERGENCY_STOP (da qualunque stato, via `/safety/lift`)

## Topic
- Pubblica: `/mower/state` (`std_msgs/String`) stato corrente
- Sottoscrive: `/mower/event` (`std_msgs/String`) eventi di transizione
- Sottoscrive: `/safety/estop` (`std_msgs/Bool`)
- Sottoscrive: `/safety/tilt` (`std_msgs/Bool`)
- Sottoscrive: `/safety/lift` (`std_msgs/Bool`)

## Parametri (in `Raspberry Pi 5/config/robot_params.yaml`)
Sezione: `mower_state_machine.ros__parameters`
- `cmd_timeout` (float, default 1.0)
- `auto_resume` (bool, default false)
- `start_on_boot` (bool, default false)
- `event_topic` (string, default `/mower/event`)
- `state_topic` (string, default `/mower/state`)
- `estop_topic` (string, default `/safety/estop`)
- `tilt_topic` (string, default `/safety/tilt`)
- `lift_topic` (string, default `/safety/lift`)

Nota: i nomi dei topic sono parametrizzati per consentire unificare la configurazione dal file globale.

## Launch
- Singolo:
```bash
ros2 launch mower_state_machine state_machine.launch.py
```
- Integrato nel bringup: `Raspberry Pi 5/bringup/launch/bringup.launch.py`

## Esempio: inviare eventi
```bash
ros2 topic pub /mower/event std_msgs/msg/String "data: 'START_MOWING'"
ros2 topic pub /mower/event std_msgs/msg/String "data: 'PAUSE'"
ros2 topic pub /mower/event std_msgs/msg/String "data: 'RESUME'"
ros2 topic pub /mower/event std_msgs/msg/String "data: 'DOCK'"
```

## Note
- Le azioni fisiche (navigazione, docking, ecc.) non sono implementate qui: questo nodo gestisce solo lo stato e gli eventi. Collegalo ai tuoi planner/controllori per eseguire i comportamenti.
