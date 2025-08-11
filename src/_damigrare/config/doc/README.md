# Smart Mower Parameter Management System

Sistema completo per la gestione dei parametri del robot tagliaerba tramite MQTT, con caricamento automatico al boot e salvataggio delle modifiche.

## Caratteristiche

- ✅ **Gestione parametri via MQTT** - Modifica parametri in tempo reale
- ✅ **Auto-calcolo tensioni batteria** - Supporto LiPo, LiFePO4, Li-ion (3S-8S)
- ✅ **Validazione parametri** - Controllo range e coerenza valori
- ✅ **Persistenza automatica** - Salvataggio automatico modifiche
- ✅ **Servizio systemd** - Avvio automatico al boot
- ✅ **Client interattivo** - Tool da linea di comando
- ✅ **Callbacks** - Notifiche per modifiche parametri critici

## Struttura File

```
src/config/
├── robot_config.json              # Configurazione parametri robot
├── parameter_manager.py           # Classe principale gestione parametri
├── mqtt_parameter_client.py       # Client MQTT per interazione
├── parameter_service.py           # Servizio per avvio automatico
├── smartmower-parameters.service  # File systemd
├── install_parameter_service.sh   # Script installazione
├── requirements.txt               # Dipendenze Python
└── README.md                      # Questa documentazione
```

## Installazione

1. **Installa il servizio:**
   ```bash
   cd /home/vito/smartmower/src/config
   chmod +x install_parameter_service.sh
   ./install_parameter_service.sh
   ```

2. **Verifica installazione:**
   ```bash
   sudo systemctl status smartmower-parameters
   ```

## Utilizzo

### Client Interattivo

```bash
# Modalità interattiva
python3 mqtt_parameter_client.py interactive

# Comandi disponibili nella modalità interattiva:
parameter> set tuning/pid/linear_kp 1.5
parameter> get
parameter> save
parameter> battery
parameter> pid
parameter> quit
```

### Comandi da Linea di Comando

```bash
# Imposta parametro
python3 mqtt_parameter_client.py set tuning/pid/linear_kp 1.5

# Ottieni configurazione completa
python3 mqtt_parameter_client.py get

# Salva configurazione
python3 mqtt_parameter_client.py save

# Carica valori di default
python3 mqtt_parameter_client.py defaults

# Valida configurazione
python3 mqtt_parameter_client.py validate
```

## Parametri Disponibili

### Parametri di Tuning (modificabili runtime)

#### PID Controller
- `tuning/pid/linear_kp` - Proporzionale controllo lineare
- `tuning/pid/linear_ki` - Integrale controllo lineare
- `tuning/pid/linear_kd` - Derivativo controllo lineare
- `tuning/pid/angular_kp` - Proporzionale controllo angolare
- `tuning/pid/angular_ki` - Integrale controllo angolare
- `tuning/pid/angular_kd` - Derivativo controllo angolare

#### Filtro Kalman
- `tuning/kalman/process_noise_pos` - Rumore processo posizione
- `tuning/kalman/process_noise_vel` - Rumore processo velocità
- `tuning/kalman/measurement_noise_gps` - Rumore misura GPS
- `tuning/kalman/measurement_noise_imu` - Rumore misura IMU

#### Velocità
- `tuning/speeds/max_linear_speed` - Velocità lineare massima (m/s)
- `tuning/speeds/max_angular_speed` - Velocità angolare massima (rad/s)
- `tuning/speeds/cutting_speed` - Velocità durante taglio (m/s)

#### Navigazione
- `tuning/navigation/waypoint_tolerance` - Tolleranza waypoint (m)
- `tuning/navigation/obstacle_detection_distance` - Distanza rilevamento ostacoli (m)
- `tuning/navigation/boundary_safety_margin` - Margine sicurezza confine (m)

#### Pattern di Taglio
- `tuning/cutting/pattern` - Tipo pattern (zigzag, spiral, random)
- `tuning/cutting/stripe_width` - Larghezza strisce (m)
- `tuning/cutting/overlap_percentage` - Sovrapposizione (%)
- `tuning/cutting/cutting_height` - Altezza taglio (mm)

### Parametri Batteria

#### Configurazione Base
- `battery/type` - Tipo batteria (LiPo, LiFePO4, Li-ion)
- `battery/cell_count` - Numero celle (3-8)
- `battery/capacity` - Capacità (mAh)
- `battery/c_rating` - C-rating scarica

#### Soglie (auto-calcolate)
- `battery/voltage_thresholds/pack_full` - Tensione piena
- `battery/voltage_thresholds/pack_nominal` - Tensione nominale
- `battery/voltage_thresholds/pack_low` - Tensione bassa
- `battery/voltage_thresholds/pack_critical` - Tensione critica

### Parametri Hardware (fissi per robot)

#### Dimensioni
- `hardware/dimensions/robot_length` - Lunghezza robot (m)
- `hardware/dimensions/robot_width` - Larghezza robot (m)
- `hardware/dimensions/wheel_base` - Distanza ruote (m)

#### Lama
- `hardware/blade/diameter` - Diametro lama (m)
- `hardware/blade/cutting_deck_width` - Larghezza piatto taglio (m)

## Topic MQTT

### Struttura Topic
```
smartmower_001/
├── config/
│   ├── tuning/pid/linear_kp          # Imposta parametro
│   ├── battery/type                  # Imposta tipo batteria
│   └── status/tuning/pid/linear_kp   # Conferma modifica
├── commands/
│   ├── save_config                   # Salva configurazione
│   ├── get_config                    # Ottieni configurazione
│   └── validate_config               # Valida configurazione
└── config/dump                       # Configurazione completa
```

### Esempi Messaggi MQTT

```bash
# Imposta PID Kp
mosquitto_pub -t "smartmower_001/config/tuning/pid/linear_kp" -m "1.5"

# Imposta batteria 6S LiPo
mosquitto_pub -t "smartmower_001/config/battery/type" -m "\"LiPo\""
mosquitto_pub -t "smartmower_001/config/battery/cell_count" -m "6"

# Salva configurazione
mosquitto_pub -t "smartmower_001/commands/save_config" -m ""

# Ottieni configurazione
mosquitto_pub -t "smartmower_001/commands/get_config" -m ""
```

## Configurazioni Batteria Comuni

### 6S LiPo (22.2V nominale)
```json
{
  "type": "LiPo",
  "cell_count": 6,
  "capacity": 5000,
  "voltage_thresholds": {
    "pack_full": 25.2,
    "pack_nominal": 22.2,
    "pack_low": 18.2,
    "pack_critical": 18.0
  }
}
```

### 4S LiFePO4 (12.8V nominale)
```json
{
  "type": "LiFePO4",
  "cell_count": 4,
  "capacity": 8000,
  "voltage_thresholds": {
    "pack_full": 14.4,
    "pack_nominal": 12.8,
    "pack_low": 10.2,
    "pack_critical": 10.0
  }
}
```

## Gestione Servizio

```bash
# Controllo stato
sudo systemctl status smartmower-parameters

# Avvio/Stop
sudo systemctl start smartmower-parameters
sudo systemctl stop smartmower-parameters
sudo systemctl restart smartmower-parameters

# Log in tempo reale
sudo journalctl -u smartmower-parameters -f

# Log completo
sudo journalctl -u smartmower-parameters --no-pager
```

## Integrazione nel Codice Robot

### Esempio Utilizzo ParameterManager

```python
from parameter_manager import ParameterManager

# Inizializza parameter manager
pm = ParameterManager("robot_config.json")

# Registra callback per modifiche PID
def on_pid_change(new_value, old_value):
    print(f"PID cambiato: {old_value} -> {new_value}")
    # Aggiorna controller PID
    
pm.register_callback("tuning/pid/linear_kp", on_pid_change)

# Leggi parametri
kp = pm.get_parameter("tuning/pid/linear_kp", default=1.0)
max_speed = pm.get_parameter("tuning/speeds/max_linear_speed", default=1.5)

# Ottieni info batteria
battery_info = pm.get_battery_info()
voltage_low = battery_info["voltages"]["pack_low"]
```

## Sicurezza

- ✅ Validazione range parametri
- ✅ Backup automatico configurazione
- ✅ Rollback modifiche non valide
- ✅ Log tutte le modifiche
- ✅ Servizio con privilegi limitati

## Troubleshooting

### Servizio non si avvia
```bash
# Controlla log
sudo journalctl -u smartmower-parameters --no-pager

# Verifica dipendenze
pip3 list | grep paho-mqtt

# Test manuale
cd /home/vito/smartmower/src/config
python3 parameter_service.py
```

### MQTT non funziona
```bash
# Verifica broker MQTT
sudo systemctl status mosquitto

# Test connessione
mosquitto_pub -t "test" -m "hello"
mosquitto_sub -t "test"
```

### Parametri non si salvano
```bash
# Verifica permessi file
ls -la robot_config.json

# Controlla spazio disco
df -h

# Test manuale salvataggio
python3 -c "
from parameter_manager import ParameterManager
pm = ParameterManager('robot_config.json')
print('Save result:', pm.save_config())
"
```

## Sviluppo Futuro

- [ ] Web interface per configurazione
- [ ] Backup automatico su cloud
- [ ] Profili configurazione multipli
- [ ] Import/export configurazioni
- [ ] Monitoraggio parametri in tempo reale
- [ ] Ottimizzazione automatica parametri
