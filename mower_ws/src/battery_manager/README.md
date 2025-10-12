# battery_manager

Nodo ROS 2 C++ che monitora lo stato della batteria e pubblica eventi/alert per la macchina a stati e la sicurezza.

Riferimento sorgente: `src/battery_manager/src/battery_manager_node.cpp`

## Topic
- Sottoscrive: `/battery` (`sensor_msgs/BatteryState`) - riceve tensione e corrente da pico_control_hardware
- Pubblica: `/mower/event` (`std_msgs/String`) — emette `LOW_BATTERY` quando la percentuale scende sotto soglia
- Pubblica: `/diagnostics` (`diagnostic_msgs/DiagnosticArray`) — messaggi di errore per batteria critica

## Modalità Utilizzata
- **`battery_state`** (ottimale): riceve BatteryState da pico_control_hardware e calcola automaticamente la percentuale quando non disponibile

*Nota: pico_control_hardware pubblica BatteryState con tensione e corrente ma senza percentage calcolata. Il battery_manager calcola la percentage dalla tensione e determina lo stato di carica dalla corrente.*

## Funzionamento
1. **Ricezione dati**: riceve tensione e corrente da pico_control_hardware via BatteryState
2. **Calcolo percentage**: se non disponibile nel messaggio, la calcola dalla tensione usando formula lineare
3. **Determinazione stato**: usa corrente per determinare se batteria è in carica/scarica
4. **Monitoraggio soglie**: genera eventi LOW_BATTERY e CRITICAL basandosi sulle soglie
5. **Arricchimento**: pubblica BatteryState completo con tutti i metadati batteria

## Parametri
- `event_topic` (string, default `/mower/event`)
- `error_topic` (string, default `/diagnostics`)
- `battery_topic` (string, default `/battery`) - riceve BatteryState da pico_control_hardware
- `battery_input_type` (string, default `battery_state`) - modalità ottimale per dati da pico_control_hardware
- `voltage_max_v` (double, default `12.6`) — tensione batteria piena (V)
- `voltage_min_v` (double, default `9.0`) — tensione batteria critica (V)
- `low_battery_threshold_pct` (double, default `30.0`) — soglia sotto cui emettere `LOW_BATTERY`
- `low_battery_clear_pct` (double, default `35.0`) — isteresi clear per `LOW_BATTERY`
- `critical_battery_threshold_pct` (double, default `5.0`) — sotto questa soglia pubblica errore critico
- `critical_battery_clear_pct` (double, default `8.0`) — isteresi clear per errore critico

## Avvio
```bash
ros2 run battery_manager battery_manager_node --ros-args --params-file <config.yaml>
```

## Integrazione
- Riceve dati batteria grezzi da `pico_control_hardware` via topic `/battery`
- Elabora automaticamente percentage e stato batteria quando non disponibili
- Invia eventi alla macchina a stati per gestione livelli batteria bassi
- Invia diagnostica al safety_supervisor per arresto emergenza su batteria critica

## Architettura
Il nodo è ottimizzato per funzionare con `pico_control_hardware` che fornisce dati batteria grezzi (tensione e corrente). Il battery_manager elabora questi dati per fornire informazioni complete sullo stato batteria, calcolando percentage e determinando stato di carica/scarica automaticamente.
