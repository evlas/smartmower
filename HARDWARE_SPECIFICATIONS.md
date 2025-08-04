# Smart Mower - Specifiche Hardware

## Configurazione Pin GPIO

### Motori
- **Motore Sinistro**:
  - PWM: GPIO0
  - DIR: GPIO1
  - ENC: GPIO2 (Encoder)
- **Motore Destro**:
  - PWM: GPIO3
  - DIR: GPIO4
  - ENC: GPIO5
- **Lama 1**:
  - PWM: GPIO6
  - DIR: GPIO7
  - ENC: GPIO8
- **Lama 2**:
  - PWM: GPIO9
  - DIR: GPIO10
  - ENC: GPIO11

### Comunicazione I2C
- SDA: GPIO12
- SCL: GPIO13

### Sensori ad Ultrasuoni
- **Anteriore Sinistro**:
  - TRIG: GPIO14
  - ECHO: GPIO15
- **Anteriore Centrale**:
  - TRIG: GPIO16
  - ECHO: GPIO17
- **Anteriore Destro**:
  - TRIG: GPIO18
  - ECHO: GPIO19

### Comunicazione UART
- TX: GPIO20
- RX: GPIO21

### Relè
- RELAY_PIN: GPIO22

### Pin Riservati
- GPIO26, GPIO27, GPIO28 (per espansioni future)

## Configurazione Alimentazione

### Alimentazione Principale
- **VSYS** (Tensione di sistema):
  - Pin 39 (VSYS) - Alimentazione principale 1.8-5.5V
  - Pin 37 (3V3_EN) - Abilitazione regolatore 3.3V

### Alimentazione Motori
- **Motori CC**:
  - VBUS (Pin 40) - Alimentazione diretta batteria (5V)
  - Collegati a ponte H tramite i pin PWM e DIR

### Alimentazione Sensori
- **3.3V**:
  - Pin 36 (3V3) - Alimentazione sensori I2C, ultrasuoni e logica
  - Capacità di erogazione: 300mA massimi

## Dispositivi I2C
- **MPU6050 (IMU)**: 0x68
- **HMC5883L (Magnetometro)**: 0x1E
- **INA226 (Monitor Potenza)**: 0x40
- **PCF8574 (Espansore GPIO)**: 0x20

## Configurazione PCF8574
L'ESPANSORE GPIO PCF8574 è collegato al bus I2C e fornisce 8 linee di I/O aggiuntive:

### Pin PCF8574
| Pin | Funzione | Tipo | Descrizione |
|-----|----------|------|-------------|
| P0  | RAIN_SENSOR | Input | Sensore pioggia (0 = pioggia rilevata) |
| P1  | BUMPER | Input | Sensore urto (0 = urto rilevato) |
| P2  | LIFT_SENSOR | Input | Sensore sollevamento (0 = robot sollevato) |
| P3  | STATUS_LED | Output | LED di stato (1 = acceso) |
| P4  | UNUSED | I/O | Non utilizzato (disponibile) |
| P5  | UNUSED | I/O | Non utilizzato (disponibile) |
| P6  | UNUSED | I/O | Non utilizzato (disponibile) |
| P7  | UNUSED | I/O | Non utilizzato (disponibile) |

### Resistori di Pull-up
- Tutti gli input hanno resistori di pull-up interni abilitati
- I pin inutilizzati sono configurati come input con pull-up

### Note
- L'indirizzo I2C predefinito è 0x20 (A0=A1=A2=0)
- I pin P4-P7 sono disponibili per espansioni future
- Lo stato dei pin può essere letto/scritto via I2C

## Analisi Consumo Energetico

### Sensori Collegati al Pico:
1. **IMU (MPU6050)**
   - Corrente tipica: ~3.6mA
   - Corrente massima: ~3.9mA

2. **Magnetometro (HMC5883L)**
   - Corrente tipica: 0.1mA
   - Corrente massima: 1mA

3. **Monitor Potenza (INA226)**
   - Corrente tipica: 1mA
   - Corrente massima: 1.5mA

4. **Sensori Ultrasonici (HC-SR04) × 3**
   - Standby: 2mA ciascuno
   - Attivo: 15mA ciascuno
   - Totale: 6mA (standby), 45mA (tutti attivi)

5. **Espansore GPIO (PCF8574)**
   - Corrente tipica: 0.1mA
   - Corrente massima: 1mA

### Consumo Totale:
- **Stato di Attesa**: ~10.8mA
- **Picco Massimo**: ~55.8mA

### Capacità Regolatore 3.3V:
- **Massima Corrente**: 300mA
- **Utilizzata dal Pico**: ~15-50mA
- **Disponibile per Periferiche**: ~250-285mA
- **Margine di Sicurezza**: ~195-229mA

## Specifiche Motori
- **Frequenza PWM**: 20kHz
- **Diametro Ruota**: 300mm
- **Diametro Lama**: 240mm
- **PPR Encoder Ruota**: 12 (impulsi per giro)
- **PPR Encoder Lama**: 6 (impulsi per giro)
- **Rapporto di Riduzione Ruota**: 185:1
- **Rapporto di Riduzione Lama**: 1:1

## Note Importanti
1. L'alimentazione principale può arrivare dal connettore micro-USB o da una sorgente esterna
2. I motori sono alimentati direttamente dalla batteria per garantire corrente sufficiente
3. La logica di controllo e i sensori sono alimentati a 3.3V
4. Il consumo totale è ben al di sotto del limite del regolatore 3.3V
5. È disponibile ampio margine per l'aggiunta di ulteriori sensori a basso consumo

## Configurazione Pin Raspberry Pi

### Connettore GPIO (Header J8)
| Pin | Funzione | Collegamento | Note |
|-----|----------|--------------|------|
| 1   | 3.3V     | Alimentazione 3.3V | Max 50mA |
| 2   | 5V       | Alimentazione 5V | |
| 3   | GPIO 2   | SDA1 (I2C) | LCD, PCF8574 |
| 4   | 5V       | Alimentazione 5V | LCD, Buzzer |
| 5   | GPIO 3   | SCL1 (I2C) | LCD, PCF8574 |
| 6   | GND      | Massa | |
| 7   | GPIO 4   | Pulsante di emergenza | Input con pull-up |
| 8   | GPIO 14  | UART TX | Al Pico RX |
| 9   | GND      | Massa | |
| 10  | GPIO 15  | UART RX | Dal Pico TX |
| 17  | 3.3V     | Alimentazione 3.3V | |
| 18  | GPIO 24  | Start/Select | Input con pull-up |
| 22  | GPIO 25  | Su/+ | Input con pull-up |
| 23  | GPIO 8   | Giu/- | Input con pull-up |
| 24  | GPIO 7   | Stop/Esc | Input con pull-up |
| 25  | GND      | Massa | |
| 27  | GPIO 0   | TXD2 | Al GPS RX (UART2) |
| 28  | GPIO 1   | RXD2 | Dal GPS TX (UART2) |
| 29  | GPIO 5   | Buzzer | Output (PWM) |

### Collegamenti UART
#### UART0 (Pico)
- **Raspberry Pi TX** (GPIO 14) → **Pico RX** (GPIO 1)
- **Raspberry Pi RX** (GPIO 15) → **Pico TX** (GPIO 0)
- **GND** → **GND** (condiviso)

#### UART2 (GPS)
- **Raspberry Pi TX** (GPIO 0) → **GPS RX**
- **Raspberry Pi RX** (GPIO 1) → **GPS TX**
- **GND** → **GND**
- **Dispositivo**: `/dev/ttyAMA2`
- **Baudrate**: 115200 bps
- **Protocollo**: NMEA

### Alimentazione
- **5V** dal connettore micro-USB o GPIO 2/4
- **3.3V** dai pin 1 o 17 per i sensori a basso consumo

### Pulsante di Emergenza
- Collegato tra **GPIO 4** e **GND**
- Resistenza di pull-up abilitata internamente
- Logica: 1 = normale, 0 = emergenza

### Display LCD I2C (PCF8574)
- **Indirizzo I2C**: 0x27 (standard) o 0x3F
- **Dimensioni**: 20x4 caratteri
- **Alimentazione**: 5V
- **Collegamenti**:
  - SDA → GPIO 2 (Pin 3)
  - SCL → GPIO 3 (Pin 5)
  - VCC → 5V (Pin 4)
  - GND → GND (Pin 6)

### Pulsanti di Controllo
| Pulsante | GPIO | Funzione |
|----------|------|----------|
| Su       | 24   | Naviga su / Aumenta valore |
| Giu      | 25   | Naviga giù / Diminuisci valore |
| Indietro | 8    | Torna indietro / Annulla |
| OK       | 7    | Conferma selezione |

### Buzzer
- **GPIO**: 5 (Pin 29)
- **Tipo**: Attivo (con oscillatore interno)
- **Tensione**: 5V
- **Corrente**: ~30mA
- **Frequenza**: 2-4KHz (controllo PWM)

### Note GPS
- Timeout di comunicazione: 1000ms
- Directory log: `/opt/smartmower/log/gps_bridge.log`
- Directory dati: `/opt/smartmower/data/gps`

## Protezioni
- Diodo di protezione su VSYS
- Fusibile sulla linea di alimentazione principale
- Condensatori di disaccoppiamento su ogni linea di alimentazione
- Protezione da sovracorrente e cortocircuito su tutte le uscite
