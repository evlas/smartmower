# safety_supervisor

Nodo di sicurezza che aggrega eventi critici e gestisce un Emergency Stop (E-Stop) latched, senza sostituire la macchina a stati principale del robot.

Riferimento sorgente: `src/safety_supervisor/src/safety_supervisor.cpp`

## Input (sottoscritti)
- `/buttons/estop` (`std_msgs/Bool`): `true` per attivare E-Stop manualmente
- `/safety/tilt` (`std_msgs/Bool`): `true` in caso di inclinazione critica
- `/safety/lift` (`std_msgs/Bool`): `true` in caso di sollevamento
- `/diagnostics` (`diagnostic_msgs/DiagnosticArray`): messaggi di diagnostica che possono attivare E-Stop se di livello ERROR

## Output (pubblicati)
- `/safety/estop` (`std_msgs/Bool`): stato E-Stop (latched) pubblicato quando cambia
- `/diagnostics_out` (`diagnostic_msgs/DiagnosticArray`): stato aggregato del sistema di sicurezza

## Servizi
- `estop_reset` (`std_srvs/Trigger`): resetta l'E-Stop se le condizioni di sicurezza sono OK
- `error_reset` (`std_srvs/Trigger`): resetta lo stato di errore

## Parametri
- `error_timeout_sec` (double, default `120.0`): timeout in secondi prima che un errore attivi l'E-Stop
- `error_level` (string, default `ERROR`): livello diagnostico minimo per attivare E-Stop (es. "ERROR", "WARN")
- `hardware_id` (string, default `"safety_supervisor"`): ID hardware per i messaggi diagnostici

## Comportamento
- E-Stop si attiva quando:
  - `buttons/estop` è `true` (immediato)
  - `safety/tilt` o `safety/lift` è `true` (immediato)
  - Riceve un messaggio diagnostico con livello >= `error_level` per più di `error_timeout_sec`

- E-Stop viene disattivato solo quando:
  - Viene chiamato `estop_reset`
  - Non ci sono più condizioni di errore attive (tilt/lift/error)

## Formato messaggi di errore
Il nodo pubblica messaggi diagnostici nel formato standard ROS2:
```yaml
header:
  stamp: {sec: 123, nanosec: 456}
  frame_id: "safety"
status:
- level: 2  # 0=OK, 1=WARN, 2=ERROR
  name: "safety_estop"
  message: "Emergency Stop attivato"
  hardware_id: "mower"
  values:
  - {key: "reason", value: "tilt_detected"}
  - {key: "duration", value: "0.0"}
```

## Note
- Tutti i topic possono essere rimappati a lancio.
- I messaggi diagnostici con livello inferiore a `error_level` vengono registrati ma non attivano E-Stop.
