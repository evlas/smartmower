# relay_manager

Nodo ROS 2 C++ che gestisce il controllo del relay principale del robot tagliaerba, ottimizzato per funzionare con pico_control_hardware.

Riferimento sorgente: `src/relay_manager/src/relay_manager.cpp`

## Topic
- Sottoscrive: `/mower/state` (`std_msgs/String`) - stato della macchina a stati
- Pubblica: `/relay/cmd` (`std_msgs/Bool`) - comandi relay per pico_control_hardware

## Funzionalità
- **Controllo basato su stato**: attiva/disattiva relay in base allo stato della macchina a stati
- **Comandi compatibili**: formato comandi ottimizzato per pico_control_hardware
- **Logica di sicurezza**: relay attivo solo negli stati appropriati

## Stati che Attivano il Relay
- `UNDOCKING` - Durante l'uscita dalla stazione di carica
- `MOWING` - Durante il taglio dell'erba
- `DOCKING` - Durante l'ingresso nella stazione di carica
- `MANUAL_CONTROL` - Durante controllo manuale

## Stati che Disattivano il Relay
- `IDLE` - Inattivo
- `CHARGING` - In carica
- `EMERGENCY_STOP` - Arresto di emergenza
- `ERROR` - Stato di errore
- `PAUSED` - In pausa

## Parametri
- `state_topic` (string, default `/mower/state`) - topic stato macchina a stati
- `relay_cmd_topic` (string, default `/relay/cmd`) - topic comandi per pico_control_hardware

## Avvio
```bash
ros2 run relay_manager relay_manager_node --ros-args --params-file <config.yaml>
```

## Integrazione con pico_control_hardware
- Riceve comandi di stato dalla macchina a stati
- Pubblica comandi relay nel formato atteso da pico_control_hardware (`/relay/cmd`)
- Controlla relay principale per alimentazione sistemi del robot

## Architettura
Il nodo è stato refactored per funzionare con `pico_control_hardware`:
- **Output**: comandi `Bool` nel formato atteso da pico_control_hardware
- **Input**: stato macchina a stati per determinare quando attivare il relay
- **Sicurezza**: relay attivo solo negli stati operativi appropriati
