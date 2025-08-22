# SmartMower Supervisor

Supervisore dei micro-servizi SmartMower. Gestisce:
- Avvio/stop/restart dei servizi (inizialmente `costmap_node`).
- API MQTT minimal per configurazione: get/set/apply senza versioning.
- Health usando i topic `.../status` dei servizi (timeout → restart).

## Build

```
make -j
```

### Prerequisiti
- Libreria MQTT: `libmosquitto` (dev)

Su Debian/Ubuntu:
```
sudo apt-get update
sudo apt-get install -y libmosquitto-dev mosquitto-clients
```

## Install

```
sudo make install
sudo make install-systemd
sudo systemctl enable --now smartmower-supervisor.service
```

## MQTT API
- `supervisor/config/get` (req) → `supervisor/config/state` (resp)
  - Req: `{ path?: "navigation.costmap" }`
- `supervisor/config/set` (req) → `supervisor/config/result`
  - Req: `{ path?: "a.b.c", value: <json>, merge?: true }`
  - Applica immediatamente (persistenza atomica) e riavvia selettivamente i servizi (per ora riavvio `costmap_node`).
- `supervisor/cmd` (req)
  - `{ action: start|stop|restart, service: "costmap_node" }`

## Health via status
Ogni servizio deve pubblicare `online=true` almeno 1 Hz su `<root>/<base>/status`.
Il supervisore considera un timeout di 5s come failure e fa restart.

## Config path override
- Env: `SMARTMOWER_CONFIG=/opt/smartmower/etc/config/robot_config.json`
- CLI: `--config /path/robot_config.json`

## Note
Il supervisor usa un client MQTT reale basato su `libmosquitto`. Assicurarsi che un broker MQTT sia raggiungibile (default `broker=localhost`, `port=1883` configurabili in `mqtt` del file di config).

### Test rapido MQTT
Assumendo `root_topic = smartmower` e broker locale:

1. Richiedi la configurazione:
```
mosquitto_sub -t 'smartmower/supervisor/config/state' &
mosquitto_pub -t 'smartmower/supervisor/config/get' -m '{}'
```
2. Modifica (merge) della configurazione:
```
mosquitto_sub -t 'smartmower/supervisor/config/result' &
mosquitto_pub -t 'smartmower/supervisor/config/set' -m '{"value": {"mqtt": {"port": 1883}}}'
```
3. Comandi al servizio costmap:
```
mosquitto_pub -t 'smartmower/supervisor/cmd' -m '{"action":"restart","service":"costmap_node"}'
```
