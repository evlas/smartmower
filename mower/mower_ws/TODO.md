# TODO progetto Robot Tagliaerba Autonomo

Questo file definisce lo stato attuale del progetto e le attività rimanenti per raggiungere gli obiettivi di navigazione autonoma e taglio automatico.

## Stato attuale (aggiornato)

### ✅ **COMPLETAMENTE IMPLEMENTATO E FUNZIONANTE**
- **pico_control_hardware**: Hardware interface completo per Raspberry Pi Pico ✅
  - Comunicazione UART avanzata con protocollo COBS
  - Controllo completo ruote differenziale
  - Lettura encoder e odometria
  - Sensori: IMU, sonar (3x), batteria, eventi hardware
  - Attuatori: lame (2 motori), relay, limiti di sicurezza

- **Sistema di sicurezza**: `safety_supervisor` + E-Stop ✅
  - Monitoraggio condizioni sicurezza
  - Arresto emergenza centralizzato
  - Integrazione con batteria e sensori

- **Macchina a stati**: `state_machine` ✅
  - Gestione stati operativi del robot
  - Eventi e transizioni di stato

- **Sensori e attuatori**: Tutti i nodi operativi ✅
  - `battery_manager`: Monitoraggio batteria
  - `blade_manager`: Controllo motori lame
  - `relay_manager`: Gestione relay
  - `rpi_gpio`: Interfaccia GPIO Raspberry Pi
  - `camera_ros`: Acquisizione camera
  - `ublox_gps`: GPS Ublox configurato
  - `events_bridge`: Bridge eventi

- **Configurazione centralizzata**: Parametri unificati ✅
  - `ros2_controllers.yaml` per ros2_control
  - File configurazione individuali per ogni nodo
  - URDF completo con hardware interface

### 🚧 **DA IMPLEMENTARE**
- **Storage aree**: `map_manager` per salvare/caricare aree di taglio
- **Registrazione perimetri**: Modalità manuale per definire aree
- **Pianificazione traiettorie**: `coverage_planner` per generare percorsi di taglio
- **Navigazione autonoma**: Integrazione Nav2 o controller custom
- **Missioni di taglio**: `mowing_mission` per orchestrazione completa
- **Rilevamento ostacoli**: Sistema avanzato di obstacle avoidance

## Obiettivo funzionale rimanente
1. **Registrazione aree**: Apprendere fino a 10 aree (perimetri) in manuale via GPS/odometria
2. **Pianificazione percorsi**: Generare traiettorie di copertura a strisce o griglia
3. **Navigazione autonoma**: Seguire traiettorie con Nav2 o controller custom
4. **Missioni complete**: Sequenza automatica goto → attiva lame → esegui percorso → ritorno

---

## Attività da implementare (priorità alta → bassa)

### 1) map_manager – Storage aree/perimetri ✅ PARZIALMENTE IMPLEMENTATO
**Stato attuale**: Framework esistente ma necessita sviluppo completo

**Da implementare:**
- Persistenza di massimo 10 aree (poligoni) in file YAML sotto `config/areas.yaml`
- API ROS2 completa (service/action) per gestione aree
- Trasformazioni WGS84 ↔ frame locale con tf2

### 2) manual_control – Registrazione perimetro ✅ PARZIALMENTE IMPLEMENTATO
**Stato attuale**: Nodo esistente ma necessita modalità registrazione

**Da implementare:**
- Modalità "record perimeter" con start/stop via service
- Subscribe a `/gps/fix` per registrazione punti
- Decimazione punti e validazione geometrica
- Preview perimetro e invio a map_manager

### 3) coverage_planner – Generazione traiettorie ❌ NON IMPLEMENTATO
**Implementare da zero:**
- Input: poligono area, larghezza taglio, strategia (parallele/ortogonali)
- Output: `nav_msgs/Path` ottimizzato
- Ottimizzazioni: rotazione poligono, gestione ostacoli

### 4) Navigazione autonoma – Controller traiettorie ❌ NON IMPLEMENTATO
**Opzioni:**
- **Nav2 integration**: Usare `FollowPath` o `NavigateThroughPoses`
- **Controller custom**: PID con limitazioni cinematiche
- Action client per missioni di navigazione

### 5) mowing_mission – Orchestrazione missione ❌ NON IMPLEMENTATO
**Sequenza automatica:**
- Naviga verso area di taglio
- Attiva lame in sicurezza
- Esegui traiettoria pianificata
- Disattiva lame e ritorno base
- Gestione errori e recovery

### 6) obstacle_detector – Rilevamento ostacoli ✅ PARZIALMENTE IMPLEMENTATO
**Da completare:**
- Integrazione sensori esistenti (sonar, bumper virtuali)
- Eventi di sicurezza per pause/riprese automatiche
- Integrazione con safety_supervisor

---

## Piano di sviluppo rimanente

### Fase 1: Storage e registrazione (2-3 settimane)
1. **map_manager**: Storage completo aree con API ROS2
2. **manual_control**: Modalità registrazione perimetri
3. **Test registrazione**: Validazione geometrica e storage

### Fase 2: Pianificazione percorsi (2-3 settimane)
1. **coverage_planner**: Algoritmi generazione traiettorie
2. **Preview percorsi**: Visualizzazione percorsi pianificati
3. **Ottimizzazioni**: Minimizzazione percorsi, gestione ostacoli

### Fase 3: Navigazione autonoma (3-4 settimane)
1. **Controller navigazione**: Nav2 o custom con safety
2. **Integrazione missioni**: Sequenza completa taglio
3. **Test navigazione**: Validazione percorsi reali

### Fase 4: Sistema completo (2-3 settimane)
1. **Missioni avanzate**: Multi-area, recovery errori
2. **Ottimizzazioni finali**: Performance e affidabilità
3. **Testing completo**: Validazione end-to-end

## Note tecniche attuali

- **Hardware**: Raspberry Pi Pico su `/dev/ttyAMA0` ✅
- **Comunicazione**: Protocollo COBS avanzato ✅
- **Controller**: ros2_control con `pico_control_hardware` ✅
- **Sicurezza**: E-Stop e monitoring batteria ✅
- **Stati**: Macchina a stati operativa ✅
- **Sensori**: IMU, sonar, batteria, GPS tutti attivi ✅

## Architettura futura

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   map_manager   │    │ coverage_planner │    │ mowing_mission  │
│  (Storage aree) │    │ (Traiettorie)    │    │ (Orchestrazione)│
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│ manual_control  │    │ Nav2/Controller  │    │ safety_supervisor│
│ (Registrazione) │    │ (Navigazione)    │    │ (Sicurezza)     │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

**Il sistema base è solido e funzionale - ora serve aggiungere l'intelligenza autonoma!** 🤖
