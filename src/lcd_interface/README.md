# Smart Mower LCD Interface

This module provides a local interface for the Smart Mower with:
- 16x2 I2C LCD display
- 4 navigation buttons (Up, Down, Left, Right)
- 1 emergency stop button
- Buzzer for audio feedback

## Features

- Displays robot state and battery percentage
- Simple menu navigation
- Emergency stop functionality
- Configurable buzzer volume
- MQTT integration for real-time updates

## Hardware Requirements

- Raspberry Pi (any model with GPIO)
- 16x2 I2C LCD display (default address 0x27)
- 6x push buttons
- Buzzer
- Resistors (for buttons)

## Installation

1. Install dependencies:
   ```bash
   sudo apt-get update
   sudo apt-get install python3-pip
   pip3 install -r requirements.txt
   ```

2. Enable I2C:
   ```bash
   sudo raspi-config
   # Navigate to Interface Options > I2C and enable
   ```

3. Install the service:
   ```bash
   sudo cp systemd/smartmower-lcd.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable smartmower-lcd
   sudo systemctl start smartmower-lcd
   ```

## Configuration

Edit `/opt/smartmower/etc/config/robot_config.json` to configure MQTT settings:

```json
{
  "system": {
    "communication": {
      "mqtt_broker_host": "localhost",
      "mqtt_broker_port": 1883,
      "mqtt_username": "mower",
      "mqtt_password": "smart"
    }
  }
}
```

## Usage

- **Emergency Button**: Immediately stops the mower
- **Navigation Buttons**: Navigate menus and adjust settings
- **Select Button**: Confirm selection

## Logs

View logs with:
```bash
journalctl -u smartmower-lcd -f
```
