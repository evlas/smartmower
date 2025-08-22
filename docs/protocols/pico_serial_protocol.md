# Protocollo di Comunicazione Seriale PICO

## Indice
1. [Panoramica](#panoramica)
2. [Configurazione UART](#configurazione-uart)
3. [Formato dei Messaggi](#formato-dei-messaggi)
4. [Tipi di Messaggio](#tipi-di-messaggio)
5. [Dettagli dei Messaggi](#dettagli-dei-messaggi)
6. [Esempi di Messaggi](#esempi-di-messaggi)
7. [Gestione Errori](#gestione-errori)
8. [Frequenza di Invio](#frequenza-di-invio)

## Panoramica
Il protocollo di comunicazione seriale del modulo PICO definisce lo scambio di dati tra il firmware PICO e il bridge MQTT. Il protocollo utilizza un formato binario strutturato per massimizzare l'efficienza della trasmissione.

## Configurazione UART
- **Baudrate**: 115200 bps
- **Bit di Dati**: 8
- **Parità**: Nessuna
- **Bit di Stop**: 1
- **Controllo di Flusso**: Nessuno

## Formato dei Messaggi

### Struttura Generale
Tutti i messaggi seguono questo formato:

| Offset | Dimensione | Descrizione          |
|--------|------------|----------------------|
| 0      | 1 byte     | Tipo di messaggio    |
| 1      | N-1 byte   | Dati specifici       |

## Tipi di Messaggio
| Tipo  | Codice | Direzione     | Descrizione               |
|-------|--------|---------------|---------------------------|
| Dati Sensori | 0x01   | PICO → Bridge | Dati dei sensori in tempo reale |
| Report di Stato | 0x02 | PICO → Bridge | Stato corrente del sistema |
| Comando Motori | 0x10  | Bridge → PICO | Impostazione velocità motori |
| Comando Sistema | 0x11 | Bridge → PICO | Comandi di sistema |

## Dettagli dei Messaggi

### Dati Sensori (0x01)
Inviati periodicamente dal PICO con i dati dei sensori a 50Hz.

**Struttura Aggiornata:**
```c
typedef struct {
    uint8_t type;           // 0x01
    uint32_t timestamp;     // Timestamp in ms
    float accel[3];         // Accelerometro [m/s²] (ax, ay, az)
    float gyro[3];          // Giroscopio [rad/s] (gx, gy, gz)
    float mag[3];           // Magnetometro [uT] (mx, my, mz)
    float us_distances[3];   // Distanze ultrasuoni [m] (left, center, right)
    float bus_voltage;      // Tensione bus [V]
    float current;          // Corrente assorbita [A]
    uint8_t safety_flags;   // Bitmask flag di sicurezza
} pico_sensor_data_t;
```

**Campi:**
- `timestamp`: Tempo in millisecondi dall'accensione
- `accel[3]`: Dati accelerometro su X, Y, Z [m/s²]
- `gyro[3]`: Velocità angolare su X, Y, Z [rad/s]
- `mag[3]`: Campo magnetico su X, Y, Z [uT]
- `us_distances[3]`: Distanze dai sensori a ultrasuoni [m]
- `bus_voltage`: Tensione del bus di potenza [V]
- `current`: Corrente assorbita totale [A]
- `safety_flags`: Bitmask dei sensori di sicurezza (vedi sezione dedicata)

### Report di Stato (0x02)
Inviato periodicamente dal PICO con lo stato corrente del sistema.

**Struttura:**
```c
typedef struct {
    uint8_t type;           // 0x02
    uint32_t timestamp;     // Timestamp in ms
    float motor_speeds[4];  // Velocità impostata motori [-1.0, 1.0]
    float motor_rpm[4];     // RPM motori misurati
    uint32_t encoder_counts[4]; // Conteggi encoder
    uint8_t system_status;  // Stato del sistema (vedi sotto)
} pico_status_report_t;
```

**Campi:**
- `motor_speeds[4]`: Velocità impostata per ogni motore [-1.0, 1.0]
- `motor_rpm[4]`: Velocità effettiva dei motori [RPM]
- `encoder_counts[4]`: Conteggio impulsi encoder per ogni motore
- `system_status`: Bitmask dello stato del sistema

## Sensori di Sicurezza

### Safety Flags (in pico_sensor_data_t)
Il campo `safety_flags` contiene i seguenti flag a 1 bit ciascuno:

| Bit | Nome         | Descrizione                          |
|-----|--------------|--------------------------------------|
| 0   | EMERGENCY    | Arresto di emergenza attivo (1=attivo)|
| 1   | RAIN         | Pioggia rilevata (1=pioggia)         |
| 2   | BUMPER       | Bumper attivato (1=urto rilevato)    |
| 3   | LIFT         | Sollevamento rilevato (1=sollevato)  |
| 4-7 | RISERVATO    | Per usi futuri (impostati a 0)       |

### System Status (in pico_status_report_t)
Il campo `system_status` contiene:

| Bit | Nome         | Descrizione                          |
|-----|--------------|--------------------------------------|
| 0   | MOTORS_ACTIVE| Motori attivi (1=attivi)             |
| 1   | RELAY_STATE  | Stato relè principale (1=attivo)     |
| 2   | LOW_BATTERY  | Batteria scarica (1=batteria bassa)  |
| 3-7 | RISERVATO    | Per usi futuri (impostati a 0)       |

### Report di Stato (0x02)
Inviato periodicamente dal PICO con lo stato corrente del sistema.

**Struttura:**
```c
typedef struct {
    uint8_t type;           // 0x02
    uint32_t timestamp;     // Timestamp in ms
    float motor_speeds[4];   // Velocità motori [-1.0, 1.0]
    float motor_rpm[4];      // RPM motori
    uint32_t encoder_counts[4]; // Conteggi encoder
    bool relay_state;        // Stato relè (true=attivo)
} pico_status_report_t;
```

**Campi:**
- `motor_speeds[4]`: Velocità impostata per ogni motore [-1.0, 1.0]
- `motor_rpm[4]`: Velocità effettiva dei motori [RPM]
- `encoder_counts[4]`: Conteggio impulsi encoder per ogni motore
- `relay_state`: Stato attuale del relè principale

### Comando Motori (0x10)
Inviato dal bridge al PICO per impostare la velocità dei motori.

**Struttura:**
```c
typedef struct {
    uint8_t type;           // 0x10
    float left_speed;       // Velocità motore sinistro [-1.0, 1.0]
    float right_speed;      // Velocità motore destro [-1.0, 1.0]
    float blade1_speed;     // Velocità lama 1 [0.0, 1.0]
    float blade2_speed;     // Velocità lama 2 [0.0, 1.0]
} pico_motor_command_t;
```

**Campi:**
- `left_speed`: Velocità motore sinistro (-1.0 = massima indietro, 1.0 = massima avanti)
- `right_speed`: Velocità motore destro
- `blade1_speed`: Velocità lama 1 (0.0 = fermo, 1.0 = massima velocità)
- `blade2_speed`: Velocità lama 2

### Comando Sistema (0x11)
Inviato dal bridge al PICO per comandi di sistema.

**Struttura:**
```c
typedef struct {
    uint8_t type;           // 0x11
    uint8_t command_id;     // ID comando
    float value;            // Valore del comando
} pico_system_command_t;
```

**Comandi Supportati:**
| ID Comando | Nome          | Valore | Descrizione                     |
|------------|---------------|--------|---------------------------------|
| 0x01       | EMERGENCY_STOP| 0.0    | Arresto di emergenza            |
| 0x02       | RESET         | 0.0    | Reset del sistema               |
| 0x03       | SET_RELAY     | 0.0/1.0| Imposta stato relè (0=off, 1=on)|

## Esempi di Messaggi

### Esempio Dati Sensori (0x01)
```
01 00 00 00 00 00 00 80 3F 00 00 00 40 00 00 40 40 00 00 80 40 00 00 A0 40 00 00 C0 40 00 00 E0 40 00 00 00 41 00 00 10 41 00 00 20 41 00 00 30 41 00 00 40 41 00 00 48 41 00 00 50 41 00 00 58 41 01
```

### Esempio Comando Motori (0x10)
```
10 00 00 80 3F 00 00 80 3F 00 00 00 00 00 00 00 00
```

## Gestione Errori
- **Checksum**: Opzionale (non implementato nella versione attuale)
- **Timeout**: 1000ms per risposta
- **Ritrasmissione**: Non prevista, i messaggi possono essere persi

## Frequenza di Invio
- **Dati Sensori**: 100Hz (ogni 10ms)
- **Report di Stato**: 1Hz (ogni 1000ms)
- **Comandi**: Su richiesta

---
*Documentazione aggiornata al 11/08/2025*
