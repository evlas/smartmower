"""
Test di connessione MQTT per il Perimeter Detector

Questo script verifica che il perimeter detector sia in ascolto sul broker MQTT
ed in grado di ricevere e rispondere ai messaggi.
"""
import paho.mqtt.client as mqtt
import json
import time
import base64
import cv2
import numpy as np
from datetime import datetime

# Add parent directory to path to import config_utils
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
from config_utils import get_mqtt_credentials_safe

# MQTT Configuration
broker_host, username, password, port = get_mqtt_credentials_safe()
print(f"Using MQTT config: {broker_host}:{port} (user: {username})")

BROKER = broker_host
PORT = port
USERNAME = username
PASSWORD = password
SUBSCRIBE_TOPIC = "smartmower/vision/perimeter"
PUBLISH_TOPIC = "smartmower/vision/camera"

# Variabili globali
message_received = False
last_message = None

def on_connect(client, userdata, flags, rc):
    """Callback per la connessione MQTT"""
    if rc == 0:
        print(f"✓ Connesso al broker MQTT su {BROKER}:{PORT}")
        client.subscribe(SUBSCRIBE_TOPIC)
        print(f"✓ In ascolto sul topic: {SUBSCRIBE_TOPIC}")
    else:
        print(f"✗ Connessione fallita con codice: {rc}")

def on_message(client, userdata, msg):
    """Callback per la ricezione dei messaggi"""
    global message_received, last_message
    try:
        data = json.loads(msg.payload.decode())
        last_message = data
        message_received = True
        
        print("\n✓ Messaggio ricevuto dal perimeter detector:")
        print(f"   Timestamp: {datetime.fromtimestamp(data['timestamp']/1000)}")
        
        if 'perimeter' in data and data['perimeter']:
            print(f"   Rilevati {len(data['perimeter'])} oggetti di perimetro")
            for i, obj in enumerate(data['perimeter']):
                print(f"   Oggetto {i+1}: {obj['width']}x{obj['height']}px "
                      f"a {obj['distance_m']:.2f}m ({obj['status']})")
        else:
            print("   Nessun perimetro rilevato")
            
    except Exception as e:
        print(f"Errore nel processare il messaggio: {e}")

def create_test_image(width=640, height=480, border_height=100):
    """Crea un'immagine di test con un bordo orizzontale"""
    img = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Sfondo verde (erba)
    img[:] = (0, 100, 0)
    
    # Aggiungi un bordo marrone in basso
    cv2.rectangle(img, (0, height-border_height), (width, height), (30, 60, 120), -1)
    
    # Aggiungi rumore per simulare l'erba
    noise = np.random.randint(0, 30, (height, width, 3), dtype=np.uint8)
    img = cv2.add(img, noise)
    
    return img

def test_mqtt_connection():
    """Test di connessione MQTT e invio di un'immagine di test"""
    global message_received
    
    # Inizializza il client MQTT
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connetti al broker
    client.username_pw_set(USERNAME, PASSWORD)
    
    try:
        print(f"Connessione al broker MQTT su {BROKER}:{PORT}...")
        client.connect(BROKER, PORT, 60)
        client.loop_start()
        
        # Attendi la connessione
        time.sleep(1)
        
        # Crea un'immagine di test
        img = create_test_image()
        
        # Converti l'immagine in JPEG
        _, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        jpg_as_text = base64.b64encode(buffer).decode('utf-8')
        
        # Crea il payload
        payload = {
            "image": jpg_as_text,
            "timestamp": int(time.time() * 1000)
        }
        
        # Pubblica l'immagine
        print(f"\nInvio immagine di test al topic: {PUBLISH_TOPIC}")
        client.publish(PUBLISH_TOPIC, json.dumps(payload))
        
        # Attendi la risposta (massimo 10 secondi)
        print("In attesa di risposta... (premi Ctrl+C per interrompere)")
        start_time = time.time()
        while not message_received and (time.time() - start_time) < 10:
            time.sleep(0.1)
        
        if not message_received:
            print("\n✗ Nessuna risposta ricevuta dal perimeter detector nei 10 secondi previsti")
            print("Verifica che:")
            print("1. Il perimeter detector sia in esecuzione")
            print("2. I topic in config.json siano corretti")
            print("3. Il broker MQTT sia raggiungibile")
        
    except KeyboardInterrupt:
        print("\nTest interrotto dall'utente")
    except Exception as e:
        print(f"\n✗ Errore durante il test: {e}")
    finally:
        client.loop_stop()
        client.disconnect()
        print("\nDisconnesso dal broker MQTT")

if __name__ == "__main__":
    test_mqtt_connection()
