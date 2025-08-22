# SmartMower Web Admin

Interfaccia web di gestione, controllo e amministrazione del robot. Include una sezione admin per visualizzare e modificare i parametri di `robot_config.json` via MQTT.

## Requisiti
- Linux
- libmicrohttpd (server HTTP)
- libmosquitto (client MQTT)

Su Debian/Ubuntu:
```
sudo apt-get update
sudo apt-get install -y libmicrohttpd-dev libmosquitto-dev mosquitto-clients
```

## Build
```
make
```
I binari verranno creati in `web/bin/` e installati in `/opt/smartmower/bin`.

## Installazione
```
sudo make install
```

Nota: l'avvio/stop del processo è gestito dal Supervisor (unico modulo che usa systemd).
Il servizio ascolta sulla porta HTTP 8080 (configurabile via variabile d'ambiente `SMARTMOWER_WEB_PORT`).

## Struttura dei topic MQTT
- GET configurazione: pubblica su `<root_topic>/supervisor/config/get` un payload (anche vuoto) e attende risposta su `<root_topic>/supervisor/config/state`.
- SET configurazione: pubblica su `<root_topic>/supervisor/config/set` un JSON `{ "value": { ... } }` e opzionalmente ascolta il risultato su `<root_topic>/supervisor/config/result`.

Le credenziali MQTT vengono lette da `/opt/smartmower/etc/config/robot_config.json`:
```
{
  "mqtt": {
    "broker": "localhost",
    "port": 1883,
    "username": "mower",
    "password": "smart",
    "root_topic": "smartmower"
  }
}
```

## Endpoints HTTP
- `GET /api/config` → ritorna la configurazione corrente (proxy via MQTT)
- `POST /api/config` → aggiorna la configurazione; body JSON `{ "value": { ... } }`
- `POST /api/supervisor/cmd` → inoltra comandi al Supervisor via MQTT su `<root>/supervisor/cmd`; body JSON `{ "action": "start|stop|restart", "service": "costmap_node" }`
- `GET /` → UI admin

## Sicurezza
- Questo esempio non implementa autenticazione HTTP. Mettere dietro reverse proxy/lan fidata o estendere con basic auth/TLS.

## Static UI
I file statici sono serviti da `web/static/`. Aprire `http://<host>:8080/`.

Tab presenti:
- MQTT, System, Hardware, Pico, GPS, SLAM, Navigation, Vision
- Supervisor: controlli Start/Stop/Restart di un servizio (es. `costmap_node`)

### Struttura UI modulare
L'interfaccia è stata modularizzata per tab:

- `static/index.html`: contiene solo la barra delle tab e un contenitore `<div id="tab-content">`.
- `static/app.js`: loader principale (ES module) che gestisce lo switch tab e carica dinamicamente i contenuti della tab.
- `static/js/core.js`: utilità condivise (render form da schema, helpers DOM, load/save config).
- `static/tabs/<nome>.html`: markup della singola tab (form, pulsanti, ecc.).
- `static/tabs/<nome>.js`: modulo con `export function init(App)` che inizializza la tab (render, event listeners).

Flusso:
1. L'utente clicca su una tab.
2. `app.js` fetch-a `static/tabs/<tab>.html` e lo inserisce in `#tab-content`.
3. `app.js` importa dinamicamente `static/tabs/<tab>.js` ed esegue `init(App)`.
