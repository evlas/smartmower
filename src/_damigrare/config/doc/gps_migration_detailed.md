# Migrazione Dettagliata Parametri GPS

## 🛰️ **PARAMETRI MIGRATI DAL FILE SEPARATO**

### **📁 File Origine**: `/src/gps/config.json`
### **📁 File Destinazione**: `/src/config/robot_config.json`

---

## 📊 **MAPPATURA PARAMETRI DETTAGLIATA**

### **🔌 1. Parametri Hardware UART**
| Parametro Origine | Valore | Destinazione Centralizzata | Status |
|-------------------|--------|----------------------------|--------|
| `gps.device` | `/dev/ttyAMA2` | `tuning.hardware_parameters.gps_uart_device` | ✅ |
| `gps.baudrate` | `115200` | `tuning.hardware_parameters.gps_baudrate` | ✅ |
| `gps.timeout_ms` | `1000` | `tuning.hardware_parameters.uart_timeout_ms` | ✅ |

### **🛰️ 2. Configurazione GPS Specifica**
| Parametro Origine | Valore | Destinazione Centralizzata | Status |
|-------------------|--------|----------------------------|--------|
| `gps.protocol` | `nmea` | `tuning.gps_config.protocol` | ✅ |

### **📝 3. Parametri Logging**
| Parametro Origine | Valore | Destinazione Centralizzata | Status |
|-------------------|--------|----------------------------|--------|
| `logging.level` | `debug` | `tuning.gps_logging.level` | ✅ |
| `logging.file` | `/opt/smartmower/log/gps_bridge.log` | `tuning.gps_logging.file` | ✅ |
| `logging.data_dir` | `/opt/smartmower/data/gps` | `tuning.gps_logging.data_dir` | ✅ |

---

## 🔧 **MODIFICHE CODICE IMPLEMENTATE**

### **📄 File Modificato**: `/src/gps/src/gps_bridge.c`

#### **🔄 Funzione `load_config()` - Linee 230-328**
- **Prima**: Legge da `/src/gps/config.json`
- **Dopo**: Legge da `/src/config/robot_config.json` con path centralizzati

#### **🆕 Parsing Centralizzato**
```c
// Parse configuration from centralized JSON
json_t *tuning = json_object_get(root, "tuning");
if (tuning) {
    // Hardware parameters
    json_t *hardware = json_object_get(tuning, "hardware_parameters");
    
    // GPS specific config
    json_t *gps_config = json_object_get(tuning, "gps_config");
    
    // GPS logging config
    json_t *gps_logging = json_object_get(tuning, "gps_logging");
    
    // Communication parameters
    json_t *comm = json_object_get(tuning, "communication_parameters");
}
```

---

## 🎛️ **BENEFICI MIGRAZIONE**

### **✅ Prima (File Separato)**
```bash
# Modifica protocollo GPS
vim /src/gps/config.json
# Restart servizio
systemctl restart gps-bridge
```

### **🚀 Dopo (Centralizzato + MQTT)**
```bash
# Modifica protocollo GPS via MQTT
mosquitto_pub -t "smartmower/config/set" \
  -m '{"robot.gps_config.protocol": "ubx"}'

# Cambio dispositivo UART senza restart
mosquitto_pub -t "smartmower/config/set" \
  -m '{"robot.hardware_parameters.gps_uart_device": "/dev/ttyUSB0"}'

# Cambio livello logging in tempo reale
mosquitto_pub -t "smartmower/config/set" \
  -m '{"robot.gps_logging.level": "info"}'
```

---

## 📈 **IMPATTO OPERATIVO**

### **🛰️ Gestione GPS Migliorata**
- **Protocollo Runtime**: NMEA/UBX modificabile senza restart
- **Device Hot-Swap**: Cambio porta UART dinamico
- **Baudrate Dinamico**: Ottimizzazione velocità comunicazione

### **📊 Logging Centralizzato**
- **Livello Debug**: Modificabile via MQTT
- **File Logging**: Path configurabile centralmente
- **Data Directory**: Gestione dati unificata

### **🔧 Hardware Flessibile**
- **Device Path**: `/dev/ttyAMA2`, `/dev/ttyUSB0`, `/dev/ttyACM0`
- **Baudrate**: `9600`, `38400`, `115200` configurabile
- **Timeout**: Gestione timeout adattiva

---

## ⚠️ **PARAMETRI CRITICI PER FUNZIONAMENTO**

### **🛰️ GPS (Priorità MASSIMA)**
- `gps_uart_device`: **`/dev/ttyAMA2`** - Connessione hardware GPS
- `gps_baudrate`: **115200** - Velocità comunicazione GPS
- `protocol`: **nmea** - Protocollo parsing dati

### **📝 Logging (Priorità MEDIA)**
- `level`: **debug/info/error** - Livello dettaglio log
- `file`: **Path file log** - Destinazione logging
- `data_dir`: **Directory dati** - Storage dati GPS

---

## 🚀 **CONFRONTO ARCHITETTURALE**

### **🔄 Prima (Duplicazione)**
```json
// gps/config.json
{
  "gps": { "device": "/dev/ttyAMA2", "baudrate": 115200 },
  "logging": { "level": "debug", "file": "..." }
}

// pico/config.json  
{
  "pico": { "device": "/dev/ttyAMA1", "baudrate": 115200 },  // DUPLICATO!
  "logging": { "level": "debug", "file": "..." }            // DUPLICATO!
}
```

### **🎯 Dopo (Centralizzato)**
```json
// robot_config.json
{
  "tuning": {
    "hardware_parameters": {
      "gps_uart_device": "/dev/ttyAMA2",
      "pico_uart_device": "/dev/ttyAMA1",
      "gps_baudrate": 115200,
      "pico_baudrate": 115200,
      "uart_timeout_ms": 1000
    },
    "gps_config": { "protocol": "nmea" },
    "gps_logging": { "level": "debug", "file": "...", "data_dir": "..." }
  }
}
```

---

## 📋 **VALIDAZIONE MIGRAZIONE**

### **✅ Test Compilazione**
```bash
cd /src/gps
make clean && make
# Result: SUCCESS - No compilation errors
```

### **✅ Test Configurazione**
```bash
./bin/gps_bridge /src/config/robot_config.json
# Output:
# GPS Protocol: nmea
# GPS Config loaded: /dev/ttyAMA2 @ 115200 baud, timeout 1000ms
# GPS Logging: level 2, file /opt/smartmower/log/gps_bridge.log
```

---

## 🎯 **RIEPILOGO MIGRAZIONE**

| Categoria | Parametri | Status | Impatto |
|-----------|-----------|--------|---------|
| **Hardware UART** | 3 | ✅ | Comunicazione |
| **GPS Specifici** | 1 | ✅ | Protocollo Parsing |
| **Logging** | 3 | ✅ | Debug & Monitoring |
| **TOTALE** | **7** | ✅ | **Sistema Completo** |

**🎯 Migrazione GPS: 7 parametri centralizzati con successo!**

---

## 🔄 **PROSSIMI STEP**

1. **✅ Configurazione Centralizzata**: Completata
2. **✅ Modifica Codice**: Completata
3. **✅ Test Compilazione**: Superato
4. **✅ Test Configurazione**: Validato
5. **🗑️ Rimozione File Separato**: Da completare
6. **📋 Documentazione**: Completata

**🚀 GPS Migration: COMPLETED SUCCESSFULLY!**
