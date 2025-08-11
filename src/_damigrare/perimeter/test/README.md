# Test del Perimeter Detector

Questa directory contiene script per testare il funzionamento del modulo di rilevamento perimetrale.

## Struttura

- `test_mqtt_connection.py` - Test di base della connessione MQTT
- `test_with_images.py` - Test con immagini locali
- `test_images/` - Directory contenente immagini di test
- `requirements.txt` - Dipendenze Python richieste

## Requisiti

```bash
pip install -r requirements.txt
```

## Esecuzione dei test

1. Assicurati che il broker MQTT sia in esecuzione:
   ```bash
   mosquitto -v
   ```

2. Avvia il perimeter detector in un altro terminale:
   ```bash
   cd ..
   ./perimeter_detector
   ```

3. Esegui i test:
   ```bash
   # Test di connessione MQTT
   python test_mqtt_connection.py
   
   # Test con immagini locali
   python test_with_images.py
   ```

## Creazione di immagini di test

Lo script `test_with_images.py` crea automaticamente alcune immagini di test nella cartella `test_images/`. Puoi aggiungere altre immagini in questa cartella per test aggiuntivi.

## Risoluzione dei problemi

Se i test falliscono, controlla:

1. Che il broker MQTT sia in esecuzione e raggiungibile
2. Che il perimeter detector sia in esecuzione e configurato correttamente
3. Che i topic in `config.json` corrispondano a quelli usati nei test
4. I log del perimeter detector per eventuali errori

Per abilitare i log di debug, modifica `config.json`:

```json
"debug": {
    "enabled": true,
    "log_level": "debug",
    "log_file": "/tmp/perimeter_detector.log",
    "show_windows": true
}
```
