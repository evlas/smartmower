# blade_manager

Nodo ROS 2 C++ che gestisce il controllo delle lame del robot tagliaerba, ottimizzato per funzionare con pico_control_hardware.

Riferimento sorgente: `src/blade_manager/src/blade_manager_node.cpp`

## Topic
- Sottoscrive: `/mower/state` (`std_msgs/String`) - stato della macchina a stati
- Pubblica: `/blades/cmd` (`std_msgs/Float32MultiArray`) - comandi lame per pico_control_hardware

## Funzionalità
- **Controllo basato su stato**: attiva/disattiva lame in base allo stato della macchina a stati
- **Comandi compatibili**: formato comandi ottimizzato per pico_control_hardware
- **Sicurezza integrata**: logica per prevenire danni da uso improprio

## Parametri
- `state_topic` (string, default `/mower/state`) - topic stato macchina a stati
- `blades_cmd_topic` (string, default `/blades/cmd`) - topic comandi per pico_control_hardware
- `enable_states` (vector<string>, default `["MOWING"]`) - stati in cui attivare le lame
- `target_speed` (double, default `0.8`) - velocità lame quando attive (0.0-1.0)
- `min_command_threshold` (double, default `0.1`) - soglia minima comandi

## Avvio
```bash
ros2 run blade_manager blade_manager_node --ros-args --params-file <config.yaml>
```

## Integrazione con pico_control_hardware
- Riceve comandi di stato dalla macchina a stati
- Pubblica comandi lame nel formato atteso da pico_control_hardware (`/blades/cmd`)
- Pronto per futura integrazione con dati RPM da pico_control_hardware per rilevazione stall

## Architettura
Il nodo è stato refactored per funzionare con `pico_control_hardware`:
- **Output**: comandi nel formato `Float32MultiArray` atteso da pico_control_hardware
- **Input**: stato macchina a stati per determinare quando attivare le lame
- **Estendibilità**: predisposto per ricevere dati RPM da pico_control_hardware per funzionalità avanzate di sicurezza
