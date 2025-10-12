# rpi_gpio

Nodo ROS 2 C++ per la gestione dei GPIO del Raspberry Pi 5 (E-Stop e pulsanti).

## Funzioni
- Legge E-Stop su GPIO configurabile (default GPIO 4, attivo-basso con pull-up)
- Legge pulsanti OK, Back, Up, Down (GPIO configurabili, attivo-basso)
- Pubblica:
  - `/buttons/estop` (`std_msgs/Bool`)
  - `/buttons/ok`, `/buttons/back`, `/buttons/up`, `/buttons/down` (`std_msgs/Bool`)

## Parametri (definiti in `Raspberry Pi 5/config/robot_params.yaml`)
Sezione: `rpi_gpio.ros__parameters`
- `chip_name`: "gpiochip0"
- `poll_rate_hz`: 200
- `debounce_ms`: 50
- `estop_pin`: 4
- `estop_active_low`: true
- `estop_topic`: "/buttons/estop"
- `button_active_low`: true
- `buttons_namespace`: "/buttons"
- `ok_pin`: 7
- `back_pin`: 8
- `up_pin`: 24
- `down_pin`: 25

## Launch
Incluso in `bringup/launch/bringup.launch.py` con `parameters=[config_path]` per caricare il file unico `robot_params.yaml`.

## Requisiti
- `libgpiod` (`sudo apt-get install -y libgpiod-dev`)

## Test rapido
```bash
ros2 topic echo /buttons/estop
ros2 topic echo /buttons/ok
```
Premi E-Stop o i pulsanti per vedere le transizioni (`true` su pressione, con debounce).
