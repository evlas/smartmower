# Smart Mower Pico Firmware

Firmware ottimizzato per Raspberry Pi Pico per il controllo di motori, sensori e comunicazione UART con Raspberry Pi 5.

## 🚀 Quick Start

### Prerequisiti
- Raspberry Pi Pico con MicroPython installato
- Thonny IDE o altro editor compatibile con MicroPython
- Cavo USB per connessione al Pico

### Installazione

1. **Connetti il Pico in modalità BOOTSEL**:
   - Tieni premuto il pulsante BOOTSEL
   - Collega il cavo USB
   - Rilascia il pulsante

2. **Installa MicroPython** (se non già presente):
   - Scarica il firmware MicroPython per Pico da [micropython.org](https://micropython.org/download/rp2-pico/)
   - Copia il file `.uf2` sul drive `RPI-RP2` che appare

3. **Carica il firmware**:
   ```bash
   # Opzione 1: Usando Thonny
   # - Apri Thonny
   # - Connetti al Pico
   # - Carica config.py come file
   # - Carica pico_main.py come main.py
   
   # Opzione 2: Usando ampy (se installato)
   ampy -p /dev/ttyACM0 put config.py
   ampy -p /dev/ttyACM0 put pico_main.py main.py
   ```

4. **Verifica installazione**:
   - Il Pico dovrebbe avviarsi automaticamente
   - LED di stato dovrebbe lampeggiare
   - Comunicazione UART attiva su GP16/GP17

## 📋 Struttura File

```
firmware/
├── config.py         # Configurazione centralizzata
├── pico_main.py      # Firmware principale  
├── main.py           # → Copia di pico_main.py (auto-start)
└── README.md         # Questa documentazione
```

## ⚙️ Configurazione

### Pin Mapping
Il firmware utilizza la seguente mappatura pin (definita in `config.py`):

#### Motori (16 pin)
- **Motor Left**: PWM=GP0, DIR=GP1, ENC=GP2
- **Motor Right**: PWM=GP3, DIR=GP4, ENC=GP5  
- **Blade 1**: PWM=GP6, DIR=GP7, ENC=GP8
- **Blade 2**: PWM=GP9, DIR=GP10, ENC=GP11

#### Sensori I2C (2 pin)
- **SDA**: GP18, **SCL**: GP19
- **Dispositivi**: MPU6050, HMC5883L, INA226, PCF8574

#### Ultrasuoni (6 pin)
- **Front Left**: TRIG=GP20, ECHO=GP21
- **Front Center**: TRIG=GP26, ECHO=GP27
- **Front Right**: TRIG=GP28, ECHO=GP22

#### Comunicazione (2 pin)
- **UART**: TX=GP16, RX=GP17 (921600 baud)

#### Altri (1 pin)
- **Relay**: GP15

**Totale**: 22/26 pin utilizzati, 4 pin liberi per espansioni future.

### Parametri Principali

```python
# Frequenza comunicazione
COMMUNICATION_FREQ_HZ = 100  # 100Hz

# PWM motori  
PWM_FREQUENCY = 20000        # 20kHz

# I2C
I2C_FREQUENCY = 400000       # 400kHz

# Sicurezza
OBSTACLE_DISTANCE_M = 0.3    # 30cm
COMMAND_TIMEOUT_MS = 5000    # 5s timeout
```

## 🔧 Personalizzazione

### Modifica Configurazione
Tutti i parametri sono configurabili nel file `config.py`:

```python
# Esempio: Cambiare frequenza comunicazione
class CommConfig:
    COMMUNICATION_FREQ_HZ = 50  # Da 100Hz a 50Hz
    
# Esempio: Cambiare distanza sicurezza
class SafetyConfig:
    OBSTACLE_DISTANCE_M = 0.5   # Da 30cm a 50cm
```

### Validazione Automatica
Il firmware include validazione automatica della configurazione:

```bash
# Test configurazione
python3 config.py
```

## 📊 Monitoraggio

### Performance
Il firmware monitora automaticamente:
- **Frequenza effettiva** del loop principale
- **Memoria libera** disponibile
- **Latenza comunicazione** UART
- **Stato sensori** di sicurezza

### Debug
Attiva il debug modificando `config.py`:

```python
class DebugConfig:
    ENABLE_UART_DEBUG = True
    ENABLE_SENSOR_DEBUG = True
    ENABLE_PERFORMANCE_MONITORING = True
```

## 🛡️ Sicurezza

### Funzioni di Sicurezza
- **Emergency Stop** automatico su:
  - Ostacoli rilevati (< 30cm)
  - Sensore bumper attivato
  - Sensore sollevamento attivato
  - Timeout comunicazione (> 5s)

### Watchdog
- Monitoraggio continuo comunicazione Pi 5
- Reset automatico motori in caso di problemi
- Gestione relay sicura (off su emergenza)

## 🔌 Connessioni Hardware

### Schema Connessioni
```
Pico Pin    → Dispositivo
GP0-GP11    → Driver motori (PWM/DIR/ENC)
GP15        → Relay controllo
GP16-GP17   → UART Pi 5
GP18-GP19   → Bus I2C sensori
GP20-GP22   → Ultrasuoni front
GP26-GP28   → Ultrasuoni front
```

### Alimentazione
- **Pico**: 5V via USB o pin VSYS
- **Motori**: Alimentazione separata tramite driver
- **Sensori**: 3.3V dal Pico (pin 3V3)

## 🚨 Troubleshooting

### Problemi Comuni

**Pico non si avvia:**
- Verifica connessione USB
- Controlla che main.py sia presente
- Reset tenendo BOOTSEL + power cycle

**Comunicazione UART non funziona:**
- Verifica baud rate (115200)
- Controlla connessioni TX/RX
- Testa con terminale seriale

**Sensori I2C non rispondono:**
- Verifica alimentazione 3.3V
- Controlla connessioni SDA/SCL
- Test indirizzi I2C con scanner

**Motori non si muovono:**
- Verifica alimentazione driver motori
- Controlla connessioni PWM/DIR
- Test segnali con oscilloscopio

### Log di Debug
```python
# Attiva tutti i log di debug
DEBUG.ENABLE_UART_DEBUG = True
DEBUG.ENABLE_SENSOR_DEBUG = True
DEBUG.ENABLE_MOTOR_DEBUG = True
```

## 📈 Performance

### Specifiche Target
- **Frequenza loop**: 100Hz (10ms periodo)
- **Latenza controllo**: < 10ms
- **CPU usage**: < 5% per core
- **Memoria libera**: > 100KB

### Ottimizzazioni
- **Dual-core**: Core 0 controllo, Core 1 sensori
- **Interrupt encoder**: Zero polling overhead
- **JSON compatto**: ~150 bytes per messaggio
- **Garbage collection**: Automatico ogni 5000 loop

## 🔄 Aggiornamenti

### Versioning
- **v1.0.0**: Release iniziale
- **v1.1.0**: Configurazione centralizzata
- **v1.2.0**: Ottimizzazioni performance

### Update Firmware
1. Backup configurazione corrente
2. Carica nuovi file via Thonny
3. Verifica compatibilità configurazione
4. Test funzionalità base

---

## 📞 Supporto

Per problemi o domande:
- Controlla la documentazione in `docs/`
- Verifica configurazione con `python3 config.py`
- Testa comunicazione con esempi in `docs/communication_protocol.md`

**Firmware Version**: v1.1.0  
**Last Updated**: 2024-01-XX
