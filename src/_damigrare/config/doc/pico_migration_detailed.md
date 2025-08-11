# Migrazione Dettagliata Parametri Pico

## 🎯 **PARAMETRI MIGRATI DAL FILE SEPARATO**

### **📁 File Origine**: `/src/pico/config.json`
### **📁 File Destinazione**: `/src/config/robot_config.json`

---

## 📊 **MAPPATURA PARAMETRI DETTAGLIATA**

### **🔌 1. Parametri Hardware UART**
| Parametro Origine | Valore | Destinazione Centralizzata | Status |
|-------------------|--------|----------------------------|--------|
| `pico.device` | `/dev/ttyAMA1` | `tuning.hardware_parameters.pico_uart_device` | ✅ |
| `pico.baudrate` | `115200` | `tuning.hardware_parameters.pico_baudrate` | ✅ |
| `pico.timeout_ms` | `1000` | `tuning.hardware_parameters.uart_timeout_ms` | ✅ |

### **🔋 2. Configurazione Batteria Base**
| Parametro Origine | Valore | Destinazione Centralizzata | Status |
|-------------------|--------|----------------------------|--------|
| `battery.type` | `lithium_ion` | `tuning.pico_battery_config.type` | ✅ |
| `battery.cells` | `6` | `tuning.pico_battery_config.cells` | ✅ |
| `battery.nominal_voltage_per_cell` | `3.7` | `tuning.pico_battery_config.nominal_voltage_per_cell` | ✅ |
| `battery.max_voltage_per_cell` | `4.2` | `tuning.pico_battery_config.max_voltage_per_cell` | ✅ |
| `battery.min_voltage_per_cell` | `3.0` | `tuning.pico_battery_config.min_voltage_per_cell` | ✅ |
| `battery.capacity_ah` | `5.0` | `tuning.pico_battery_config.capacity_ah` | ✅ |

### **⚡ 3. Parametri Ricarica**
| Parametro Origine | Valore | Destinazione Centralizzata | Status |
|-------------------|--------|----------------------------|--------|
| `battery.charging.full_voltage_per_cell` | `4.15` | `tuning.pico_battery_config.charging.full_voltage_per_cell` | ✅ |
| `battery.charging.trickle_current_ma` | `100` | `tuning.pico_battery_config.charging.trickle_current_ma` | ✅ |
| `battery.charging.stable_time_seconds` | `300` | `tuning.pico_battery_config.charging.stable_time_seconds` | ✅ |
| `battery.charging.current_threshold_ma` | `50` | `tuning.pico_battery_config.charging.current_threshold_ma` | ✅ |

### **📊 4. Curva Tensione Batteria (12 punti)**
| Percentuale | Tensione | Destinazione Centralizzata | Status |
|-------------|----------|----------------------------|--------|
| `100%` | `4.2V` | `tuning.pico_battery_config.voltage_curve.100` | ✅ |
| `90%` | `4.1V` | `tuning.pico_battery_config.voltage_curve.90` | ✅ |
| `80%` | `4.0V` | `tuning.pico_battery_config.voltage_curve.80` | ✅ |
| `70%` | `3.9V` | `tuning.pico_battery_config.voltage_curve.70` | ✅ |
| `60%` | `3.8V` | `tuning.pico_battery_config.voltage_curve.60` | ✅ |
| `50%` | `3.75V` | `tuning.pico_battery_config.voltage_curve.50` | ✅ |
| `40%` | `3.7V` | `tuning.pico_battery_config.voltage_curve.40` | ✅ |
| `30%` | `3.65V` | `tuning.pico_battery_config.voltage_curve.30` | ✅ |
| `20%` | `3.6V` | `tuning.pico_battery_config.voltage_curve.20` | ✅ |
| `10%` | `3.4V` | `tuning.pico_battery_config.voltage_curve.10` | ✅ |
| `5%` | `3.2V` | `tuning.pico_battery_config.voltage_curve.5` | ✅ |
| `0%` | `3.0V` | `tuning.pico_battery_config.voltage_curve.0` | ✅ |

### **📝 5. Parametri Logging**
| Parametro Origine | Valore | Destinazione Centralizzata | Status |
|-------------------|--------|----------------------------|--------|
| `logging.level` | `debug` | `tuning.pico_logging.level` | ✅ |
| `logging.file` | `/opt/smartmower/log/pico_bridge.log` | `tuning.pico_logging.file` | ✅ |
| `logging.data_dir` | `/opt/smartmower/data/pico` | `tuning.pico_logging.data_dir` | ✅ |

---

## 🔧 **MODIFICHE CODICE NECESSARIE**

### **📄 File da Modificare**: `/src/pico/src/pico_bridge.c`

#### **🔄 Funzione `load_config()` - Linee 678-826**
- **Prima**: Legge da `/src/pico/config.json`
- **Dopo**: Legge da `/src/config/robot_config.json` con path centralizzati

#### **🔄 Struttura `config_t` - Include header**
- Aggiornare per compatibilità con nuovi path JSON

---

## 🎛️ **BENEFICI MIGRAZIONE**

### **✅ Prima (File Separato)**
```bash
# Modifica parametri batteria
vim /src/pico/config.json
# Restart servizio
systemctl restart pico-bridge
```

### **🚀 Dopo (Centralizzato + MQTT)**
```bash
# Modifica tipo batteria via MQTT
mosquitto_pub -t "smartmower/config/set" \
  -m '{"robot.pico_battery_config.type": "lithium_phosphate"}'

# Modifica curva tensione in tempo reale
mosquitto_pub -t "smartmower/config/set" \
  -m '{"robot.pico_battery_config.voltage_curve.50": 3.8}'

# Cambio dispositivo UART senza restart
mosquitto_pub -t "smartmower/config/set" \
  -m '{"robot.hardware_params.pico_uart_device": "/dev/ttyACM0"}'
```

---

## 📈 **IMPATTO OPERATIVO**

### **🔋 Gestione Batteria Migliorata**
- **Tuning Runtime**: Curve tensione modificabili senza restart
- **Multi-Chimica**: Supporto LiPo/LiFePO4/Li-Ion configurabile
- **Calibrazione**: Parametri ricarica ottimizzabili in campo

### **🔧 Hardware Flessibile**
- **Device Hot-Swap**: Cambio porta UART senza ricompilazione
- **Baudrate Dinamico**: Ottimizzazione velocità comunicazione
- **Timeout Adattivo**: Gestione timeout basata su condizioni

### **📊 Monitoring Centralizzato**
- **Logging Unificato**: Tutti i log in configurazione centrale
- **Debug Dinamico**: Livello log modificabile via MQTT
- **Telemetria**: Parametri batteria visibili in dashboard

---

## ⚠️ **PARAMETRI CRITICI PER SICUREZZA**

### **🔋 Batteria (Priorità MASSIMA)**
- `min_voltage_per_cell`: **3.0V** - Protezione scarica profonda
- `max_voltage_per_cell`: **4.2V** - Protezione sovraccarica
- `current_threshold_ma`: **50mA** - Rilevamento carica completa

### **🔌 Comunicazione (Priorità ALTA)**
- `pico_uart_device`: **`/dev/ttyAMA1`** - Connessione hardware
- `pico_baudrate`: **115200** - Velocità comunicazione
- `uart_timeout_ms`: **1000ms** - Timeout sicurezza

---

## 🚀 **PROSSIMI STEP**

1. **✅ Configurazione Centralizzata**: Completata
2. **🔄 Modifica Codice**: In corso
3. **🧪 Test Funzionalità**: Da fare
4. **🗑️ Rimozione File Separato**: Finale
5. **📋 Documentazione**: Aggiornamento

---

## 📊 **RIEPILOGO MIGRAZIONE**

| Categoria | Parametri | Status | Impatto |
|-----------|-----------|--------|---------|
| **Hardware UART** | 3 | ✅ | Comunicazione |
| **Batteria Base** | 6 | ✅ | Sicurezza Critica |
| **Ricarica** | 4 | ✅ | Protezione Hardware |
| **Curva Tensione** | 12 | ✅ | Precisione SoC |
| **Logging** | 3 | ✅ | Debug & Monitoring |
| **TOTALE** | **28** | ✅ | **Sistema Completo** |

**🎯 Migrazione Pico: 28 parametri centralizzati con successo!**
