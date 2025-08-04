# MQTT Definitions System

Sistema centralizzato per la gestione delle definizioni MQTT del progetto Smart Mower.

## Struttura

```
mqtt_definitions/
├── mqtt_definitions.json      # File master con tutte le definizioni MQTT
├── generate_headers.py        # Script per generare header C/C++
├── README.md                  # Questa documentazione
└── ../generated_headers/      # Directory con gli header generati
    ├── pico_bridge_mqtt.h     # Header per pico_bridge
    ├── state_machine_mqtt.h   # Header per state_machine
    ├── slam_node_mqtt.h       # Header per slam_node
    ├── gps_bridge_mqtt.h      # Header per gps_bridge
    ├── fusion_sensor_mqtt.h   # Header per fusion_sensor
    ├── vision_system_mqtt.h   # Header per vision_system
    └── web_interface_mqtt.h   # Header per web_interface
```

## Utilizzo

### Generazione automatica degli header

```bash
cd /home/vito/smartmower/mqtt_definitions
python3 generate_headers.py
```

Questo comando genera automaticamente tutti gli header per i diversi eseguibili.

### Generazione personalizzata

```bash
# Genera solo per un eseguibile specifico
python3 generate_headers.py mqtt_definitions.json custom_output_dir
```

### Inclusione negli eseguibili

Ogni eseguibile può includere il proprio header specifico:

```c
// In pico_bridge.c
#include "../generated_headers/pico_bridge_mqtt.h"

// In state_machine.c  
#include "../generated_headers/state_machine_mqtt.h"

// In slam_node.cpp
#include "../generated_headers/slam_node_mqtt.h"
```

## Contenuto degli header generati

Ogni header contiene solo le sezioni necessarie per il rispettivo eseguibile:

### pico_bridge_mqtt.h
- ✅ Topic MQTT
- ✅ Tipi di messaggio
- ✅ Comandi sistema
- ✅ Strutture binarie
- ✅ Configurazione

### state_machine_mqtt.h
- ✅ Topic MQTT
- ✅ Tipi di messaggio
- ✅ Comandi sistema
- ✅ Configurazione

### slam_node_mqtt.h
- ✅ Topic MQTT
- ✅ Tipi di messaggio
- ✅ Configurazione

### gps_bridge_mqtt.h
- ✅ Topic MQTT
- ✅ Tipi di messaggio
- ✅ Configurazione

### fusion_sensor_mqtt.h
- ✅ Topic MQTT
- ✅ Tipi di messaggio
- ✅ Configurazione

### vision_system_mqtt.h
- ✅ Topic MQTT
- ✅ Tipi di messaggio
- ✅ Configurazione

### web_interface_mqtt.h
- ✅ Topic MQTT
- ✅ Formati messaggi JSON
- ✅ Configurazione

## Modifica delle definizioni

1. **Modifica il file master**: Edita `mqtt_definitions.json`
2. **Rigenera gli header**: Esegui `python3 generate_headers.py`
3. **Ricompila gli eseguibili**: Gli header aggiornati saranno automaticamente inclusi

## Esempio di utilizzo nel codice

```c
#include "pico_bridge_mqtt.h"

// Costruzione topic
char topic[512];
snprintf(topic, sizeof(topic), "%s%s", 
         MQTT_DEFAULT_BASE_TOPIC, 
         MQTT_TOPIC_SENSORS);

// Invio comando sistema
system_command_t cmd = {
    .type = MSG_SYSTEM_CMD,
    .command_id = SYS_CMD_EMERGENCY_STOP,
    .value = 0.0
};

// Controllo tipo messaggio JSON
if (strcmp(msg_type, JSON_MSG_TYPE_SENSOR_DATA) == 0) {
    // Processa dati sensori
}
```

## Vantaggi del sistema

1. **Centralizzazione**: Tutte le definizioni MQTT in un unico file
2. **Consistenza**: Stessi valori garantiti in tutti gli eseguibili
3. **Manutenibilità**: Modifiche in un solo posto
4. **Ottimizzazione**: Ogni eseguibile include solo ciò che serve
5. **Documentazione**: Header auto-documentati con formati JSON
6. **Validazione**: Controllo della sintassi JSON automatico

## Aggiunta di nuovi eseguibili

Per aggiungere supporto per un nuovo eseguibile, modifica la lista `executables` in `generate_headers.py`:

```python
executables = [
    # ... eseguibili esistenti ...
    {
        'name': 'nuovo_eseguibile',
        'sections': ['topics', 'message_types', 'config']  # Sezioni necessarie
    }
]
```

## Troubleshooting

### Errore "Definitions file not found"
Assicurati di essere nella directory corretta e che `mqtt_definitions.json` esista.

### Header non aggiornati
Dopo aver modificato `mqtt_definitions.json`, ricorda di rigenerare gli header con:
```bash
python3 generate_headers.py
```

### Errori di compilazione
Verifica che il path dell'include sia corretto nel tuo Makefile:
```makefile
CFLAGS += -I../generated_headers
```
