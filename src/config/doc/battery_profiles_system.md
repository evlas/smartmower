# Sistema Profili Batteria Modulari

## 🔋 **ARCHITETTURA PROFILI BATTERIA**

### **🎯 Approccio Modulare**
Invece di duplicare i parametri batteria in ogni modulo, abbiamo creato un sistema di **profili standardizzati** riutilizzabili.

```json
{
  "battery_profiles": {
    "lipo_3s": { ... },
    "lipo_4s": { ... },
    "lipo_6s": { ... },
    "lifepo4_4s": { ... },
    "liion_6s": { ... },
    "liion_8s": { ... }
  },
  "tuning": {
    "pico_config": {
      "battery_profile": "liion_6s"  // ← Riferimento per nome
    }
  }
}
```

---

## 📊 **PROFILI BATTERIA DISPONIBILI**

### **🔥 LiPo (Lithium Polymer)**
| Profilo | Celle | Tensione Nominale | Capacità | Uso Tipico |
|---------|-------|-------------------|----------|------------|
| `lipo_3s` | 3S | 11.1V | 2.2Ah | Robot piccoli |
| `lipo_4s` | 4S | 14.8V | 3.0Ah | Robot medi |
| `lipo_6s` | 6S | 22.2V | 5.0Ah | Robot grandi |

### **⚡ LiFePO4 (Lithium Iron Phosphate)**
| Profilo | Celle | Tensione Nominale | Capacità | Uso Tipico |
|---------|-------|-------------------|----------|------------|
| `lifepo4_4s` | 4S | 12.8V | 4.0Ah | Sicurezza massima |

### **🔋 Li-Ion (Lithium Ion)**
| Profilo | Celle | Tensione Nominale | Capacità | Uso Tipico |
|---------|-------|-------------------|----------|------------|
| `liion_6s` | 6S | 22.2V | 5.0Ah | **Configurazione attuale** |
| `liion_8s` | 8S | 29.6V | 6.0Ah | Robot potenti |

---

## 🔧 **PARAMETRI PER PROFILO**

### **📊 Parametri Base**
```json
{
  "type": "lithium_ion",
  "cells": 6,
  "nominal_voltage_per_cell": 3.7,
  "max_voltage_per_cell": 4.2,
  "min_voltage_per_cell": 3.0,
  "capacity_ah": 5.0
}
```

### **⚡ Parametri Ricarica**
```json
{
  "charging": {
    "full_voltage_per_cell": 4.15,
    "trickle_current_ma": 100,
    "stable_time_seconds": 300,
    "current_threshold_ma": 50
  }
}
```

### **📈 Curva Tensione (12 punti)**
```json
{
  "voltage_curve": {
    "100": 4.2, "90": 4.1, "80": 4.0, "70": 3.9, "60": 3.8,
    "50": 3.75, "40": 3.7, "30": 3.65, "20": 3.6, "10": 3.4, 
    "5": 3.2, "0": 3.0
  }
}
```

---

## 🚀 **VANTAGGI SISTEMA MODULARE**

### **✅ Prima (Duplicazione)**
```json
// pico/config.json
{
  "battery": { "type": "lithium_ion", "cells": 6, ... }
}

// gps/config.json  
{
  "battery": { "type": "lithium_ion", "cells": 6, ... }  // DUPLICATO!
}

// fusion/config.json
{
  "battery": { "type": "lithium_ion", "cells": 6, ... }  // DUPLICATO!
}
```

### **🎯 Dopo (Modulare)**
```json
// robot_config.json
{
  "battery_profiles": {
    "liion_6s": { /* definizione completa */ }
  },
  "tuning": {
    "pico_config": { "battery_profile": "liion_6s" },
    "gps_config": { "battery_profile": "liion_6s" },
    "fusion_config": { "battery_profile": "liion_6s" }
  }
}
```

---

## 🔄 **CAMBIO BATTERIA SEMPLIFICATO**

### **🎛️ Cambio Tipo Batteria**
```bash
# Cambio da Li-Ion 6S a LiFePO4 4S
mosquitto_pub -t "smartmower/config/set" \
  -m '{"robot.pico_config.battery_profile": "lifepo4_4s"}'

# Automaticamente applica:
# - Tensioni corrette (3.2V nominale vs 3.7V)
# - Curve di scarica LiFePO4
# - Parametri ricarica specifici
# - Protezioni appropriate
```

### **⚙️ Aggiunta Nuovo Profilo**
```bash
# Aggiungi nuovo profilo personalizzato
mosquitto_pub -t "smartmower/config/set" -m '{
  "battery_profiles.custom_5s": {
    "type": "lithium_polymer",
    "cells": 5,
    "nominal_voltage_per_cell": 3.7,
    "max_voltage_per_cell": 4.2,
    "min_voltage_per_cell": 3.0,
    "capacity_ah": 4.5,
    "charging": {
      "full_voltage_per_cell": 4.15,
      "trickle_current_ma": 130,
      "stable_time_seconds": 350,
      "current_threshold_ma": 65
    }
  }
}'
```

---

## 🔍 **DIFFERENZE TRA CHIMICHE**

### **⚡ LiPo vs Li-Ion vs LiFePO4**

| Parametro | LiPo | Li-Ion | LiFePO4 |
|-----------|------|--------|---------|
| **Tensione Nominale** | 3.7V | 3.7V | **3.2V** |
| **Tensione Max** | 4.2V | 4.2V | **3.6V** |
| **Tensione Min** | 3.0V | 3.0V | **2.5V** |
| **Densità Energia** | Alta | Media | Bassa |
| **Sicurezza** | Media | Media | **Massima** |
| **Cicli Vita** | 500-800 | 800-1200 | **2000+** |
| **Costo** | Medio | Basso | Alto |

### **🎯 Raccomandazioni d'Uso**

- **LiPo**: Massime prestazioni, peso ridotto
- **Li-Ion**: Equilibrio prestazioni/costo
- **LiFePO4**: Massima sicurezza e durata

---

## 📋 **CONFIGURAZIONE PICO AGGIORNATA**

### **🔄 Prima (Inline)**
```json
{
  "pico_battery_config": {
    "type": "lithium_ion",
    "cells": 6,
    // ... 25+ parametri duplicati
  }
}
```

### **✅ Dopo (Reference)**
```json
{
  "pico_config": {
    "battery_profile": "liion_6s"  // ← Solo 1 riferimento!
  }
}
```

---

## 🚀 **PROSSIMI STEP IMPLEMENTAZIONE**

1. **✅ Profili Batteria**: Completati
2. **🔄 Modifica Codice Pico**: Caricamento da profilo
3. **🧪 Test Funzionalità**: Verifica caricamento
4. **🗑️ Pulizia Config**: Rimozione file separati
5. **📋 Documentazione**: Aggiornamento finale

---

## 🎯 **BENEFICI FINALI**

### **🔧 Manutenzione**
- **1 posto** per modificare parametri batteria
- **Coerenza** garantita tra tutti i moduli
- **Validazione** centralizzata

### **🎛️ Flessibilità**
- **Cambio batteria** con 1 comando MQTT
- **Profili personalizzati** facilmente aggiungibili
- **Test A/B** tra diverse configurazioni

### **🛡️ Sicurezza**
- **Parametri validati** per ogni chimica
- **Protezioni specifiche** per tipo batteria
- **Prevenzione errori** di configurazione

**🎯 Sistema Profili Batteria: Modulare, Sicuro, Flessibile!**
