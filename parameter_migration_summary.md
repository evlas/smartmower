# Analisi Migrazione Parametri - Smart Mower

## RIEPILOGO ESECUTIVO

**Totale Parametri Analizzati: 147**

| Categoria | Quantità | % |
|-----------|----------|---|
| ✅ **Già Esistenti** | 23 | 15.6% |
| 🔄 **Migrabili** | 101 | 68.7% |
| ⚠️ **Parziali** | 15 | 10.2% |
| ❌ **Non Migrabili** | 8 | 5.4% |

## PARAMETRI PER MODULO

### 1. FUSION MODULE (25 parametri)
- ✅ Esistenti: 3 (wheel_base, encoder_pulses, process_noise)
- 🔄 Migrabili: 22 (IMU calibration, GPS settings, fusion parameters)
- ❌ Non migrabili: 0

### 2. GPS MODULE (7 parametri)  
- ✅ Esistenti: 0
- 🔄 Migrabili: 4 (baudrate, protocol, timeout)
- ❌ Non migrabili: 3 (device path, log files)

### 3. PICO MODULE (17 parametri)
- ✅ Esistenti: 4 (battery type, cells, capacity, voltages)
- 🔄 Migrabili: 11 (charging parameters, voltage curve)
- ❌ Non migrabili: 2 (device path, logs)

### 4. VISION CAMERA (10 parametri)
- ✅ Esistenti: 0  
- 🔄 Migrabili: 8 (resolution, fps, rotation, quality)
- ❌ Non migrabili: 2 (device paths)

### 5. VISION GRASS (7 parametri)
- ✅ Esistenti: 0
- 🔄 Migrabili: 6 (HSV thresholds, area detection)
- ❌ Non migrabili: 1 (log files)

### 6. VISION OBSTACLE (25 parametri)
- ✅ Esistenti: 1 (detection_distance)
- 🔄 Migrabili: 23 (SfM parameters, camera calibration)
- ❌ Non migrabili: 1 (log files)

### 7. VISION PERIMETER (12 parametri)
- ✅ Esistenti: 0
- 🔄 Migrabili: 11 (detection thresholds, image processing)
- ❌ Non migrabili: 1 (log files)

### 8. STATE MACHINE (20 parametri)
- ✅ Esistenti: 12 (battery, navigation, speeds, cutting)
- 🔄 Migrabili: 8 (obstacle detection, dock position)
- ❌ Non migrabili: 0

### 9. PATH PLANNING (18 parametri)
- ✅ Esistenti: 6 (area, cutting_width, overlap, speeds)
- 🔄 Migrabili: 12 (area config, permanent obstacles)
- ❌ Non migrabili: 0

### 10. SLAM MODULE (16 parametri)
- ✅ Esistenti: 2 (wheel_base, encoder_ticks)
- 🔄 Migrabili: 14 (mapping, localization, sensors)
- ❌ Non migrabili: 0

## PRIORITÀ DI MIGRAZIONE

### 🔥 **ALTA PRIORITÀ (65 parametri)**
**Tuning Parameters - Critici per Performance**
- Parametri visione: obstacle/grass/perimeter detection (35)
- Parametri sensori: IMU/GPS calibration (15) 
- Parametri SLAM: mapping/localization (10)
- Parametri fusion: noise/filter (5)

### 🔶 **MEDIA PRIORITÀ (25 parametri)**  
**Hardware Parameters - Consolidamento Duplicati**
- Configurazione camera: risoluzione, focal_length (8)
- Configurazione sensori: range, offset (12)
- Configurazione area: dimensioni, dock (5)

### 🔵 **BASSA PRIORITÀ (11 parametri)**
**System Parameters - Standardizzazione**
- Rate aggiornamento/pubblicazione (6)
- Configurazioni MQTT/timeout (5)

## STRUTTURA ESTENSIONE CONFIG CENTRALIZZATO

```json
{
  "tuning": {
    "sensors": {
      "imu_calibration": {...},
      "gps_settings": {...},
      "sonar_config": {...}
    },
    "vision": {
      "obstacle_detection": {...},
      "grass_detection": {...}, 
      "perimeter_detection": {...}
    },
    "slam": {
      "mapping": {...},
      "localization": {...}
    },
    "fusion": {
      "filter_parameters": {...}
    }
  },
  "hardware": {
    "camera": {
      "resolution": {...},
      "calibration": {...}
    },
    "sensors": {
      "sonar": {...},
      "imu": {...}
    },
    "area": {
      "boundaries": {...},
      "obstacles": {...}
    },
    "dock": {
      "position": {...}
    }
  },
  "system": {
    "rates": {...},
    "communication": {...}
  }
}
```

## BENEFICI MIGRAZIONE

✅ **Configurazione Centralizzata** - Un solo punto di controllo  
✅ **Eliminazione Duplicati** - 15 parametri duplicati consolidati  
✅ **Tuning Semplificato** - Interfaccia MQTT unificata  
✅ **Validazione Coerente** - Range checking centralizzato  
✅ **Backup/Restore** - Configurazione completa in un file  

## RACCOMANDAZIONI

1. **Fase 1**: Migrare parametri tuning critici (obstacle detection, PID)
2. **Fase 2**: Consolidare parametri hardware duplicati  
3. **Fase 3**: Standardizzare parametri sistema
4. **Fase 4**: Implementare validazione incrociata parametri
5. **Fase 5**: Tool migrazione automatica config esistenti
