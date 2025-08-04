# Pico MQTT Bridge

Bridge MQTT per il firmware Smart Mower Raspberry Pi Pico.

## Funzionalità

- **Comunicazione UART**: Connessione a 921600 baud con il Pico
- **Bridge MQTT**: Pubblica dati sensori e riceve comandi
- **Multi-threading**: Thread separato per lettura UART
- **JSON Protocol**: Parsing automatico messaggi JSON dal firmware

## Compilazione

```bash
# Installa dipendenze (Ubuntu/Debian)
sudo apt-get install libmosquitto-dev libjansson-dev

# Compila
make

# Installa (opzionale)
sudo make install
```

## Configurazione

Modifica `pico_bridge_config.json`:

```json
{
    "uart_device": "/dev/ttyUSB0",
    "baudrate": 921600,
    "mqtt": {
        "host": "localhost",
        "port": 1883,
        "base_topic": "mower/pico"
    }
}
```

## Utilizzo

```bash
# Avvio manuale
./pico_bridge

# Con configurazione personalizzata
./pico_bridge -c custom_config.json
```

## Topic MQTT

### Pubblicazione (dal Pico)
- `mower/pico/sensors` - Dati sensori (IMU, ultrasuoni, encoder)
- `mower/pico/status` - Status report del sistema

### Sottoscrizione (comandi al Pico)
- `mower/pico/cmd/motors` - Comandi motori
- `mower/pico/cmd/system` - Comandi sistema

## Esempi Comandi

### Controllo Motori
```bash
# Avanti al 50%
mosquitto_pub -t "mower/pico/cmd/motors" -m '{"type":"motor_control","left_speed":50,"right_speed":50,"blade1_speed":0,"blade2_speed":0}'

# Attiva lame
mosquitto_pub -t "mower/pico/cmd/motors" -m '{"type":"motor_control","left_speed":0,"right_speed":0,"blade1_speed":3000,"blade2_speed":3000}'
```

### Comandi Sistema
```bash
# Emergency stop
mosquitto_pub -t "mower/pico/cmd/system" -m '{"type":"system_command","command":"emergency_stop"}'

# Reset encoder
mosquitto_pub -t "mower/pico/cmd/system" -m '{"type":"system_command","command":"reset_encoders"}'
```

## Servizio Systemd

Crea `/etc/systemd/system/pico-bridge.service`:

```ini
[Unit]
Description=Smart Mower Pico MQTT Bridge
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/mower/src/pico
ExecStart=/home/pi/mower/src/pico/pico_bridge
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Attivazione:
```bash
sudo systemctl enable pico-bridge.service
sudo systemctl start pico-bridge.service
```
