# GPS Bridge

A lightweight C application that bridges a physical GPS device to MQTT. This component is part of the Smart Mower project and is designed to run on the Raspberry Pi 5.

## Features

- Reads NMEA data from a serial GPS device
- Parses common NMEA sentences (GGA, RMC, VTG)
- Publishes GPS data to MQTT topics
- Configurable via JSON configuration file
- Efficient memory usage and low CPU footprint
- Clean shutdown on SIGINT/SIGTERM

## Dependencies

- libmosquitto (MQTT client library)
- json-c (JSON parsing)
- Standard C libraries

## Building

1. Install dependencies:
   ```bash
   sudo apt-get install libmosquitto-dev libjson-c-dev
   ```

2. Build the application:
   ```bash
   cd src/gps
   make
   ```

## Configuration

Edit `../config/gps_bridge_config.json` to configure:

- GPS device path and baud rate
- MQTT broker connection details
- Logging settings

Example configuration:
```json
{
    "gps": {
        "device": "/dev/ttyUSB0",
        "baudrate": 9600,
        "protocol": "nmea",
        "timeout_ms": 1000
    },
    "mqtt": {
        "broker": "localhost",
        "port": 1883,
        "username": "mower",
        "password": "smart",
        "base_topic": "smartmower/gps",
        "client_id": "gps_bridge"
    },
    "logging": {
        "level": "info",
        "file": "/var/log/gps_bridge.log"
    }
}
```

## Installation

To install system-wide:

```bash
sudo make install
```

This will:
- Install the binary to `/usr/local/bin/gps_bridge`
- Install the default config to `/etc/gps_bridge_config.json`

## Running

```bash
gps_bridge [config_file]
```

If no config file is specified, it will look for the config at `/etc/gps_bridge_config.json`.

## MQTT Topics

GPS data is published to the following topics (relative to the base topic):

- `fix` - Raw GGA NMEA sentences
- `rmc` - Raw RMC NMEA sentences
- `vtg` - Raw VTG NMEA sentences

## Systemd Service

To run as a system service, create a systemd service file:

```ini
[Unit]
Description=GPS Bridge for Smart Mower
After=network.target mosquitto.service

[Service]
Type=simple
User=root
ExecStart=/usr/local/bin/gps_bridge /etc/gps_bridge_config.json
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Save as `/etc/systemd/system/gps-bridge.service` and enable with:

```bash
sudo systemctl daemon-reload
sudo systemctl enable gps-bridge
sudo systemctl start gps-bridge
```

## Debugging

Run with verbose output:

```bash
gps_bridge -v
```

Check system logs:

```bash
journalctl -u gps-bridge -f
```

## License

Part of the Smart Mower project. See project root for license information.
