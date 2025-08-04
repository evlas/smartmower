"""
Test semplificato per il Perimeter Detector
"""
import cv2
import numpy as np
import paho.mqtt.client as mqtt
import json
import time
import base64
import sys
import os

# Add parent directory to path to import config_utils
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
from config_utils import get_mqtt_credentials_safe

# MQTT Configuration
broker_host, username, password, port = get_mqtt_credentials_safe()
print(f"Using MQTT config: {broker_host}:{port} (user: {username})")

BROKER = broker_host
PORT = port
USERNAME = username
PASSWORD = password
PUBLISH_TOPIC = "smartmower/vision/camera"

# Crea un'immagine di test semplice
def create_test_image():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:] = (0, 100, 0)  # Sfondo verde
    cv2.rectangle(img, (100, 300), (540, 480), (30, 60, 120), -1)  # Bordo marrone
    return img

def main():
    # Crea il client MQTT
    client = mqtt.Client()
    client.username_pw_set(USERNAME, PASSWORD)
    
    print(f"Connessione a {BROKER}:{PORT}...")
    client.connect(BROKER, PORT, 60)
    client.loop_start()
    
    try:
        # Crea e mostra l'immagine di test
        img = create_test_image()
        cv2.imshow("Test Image", img)
        cv2.waitKey(1000)  # Mostra per 1 secondo
        
        # Converti in JPEG
        _, buffer = cv2.imencode('.jpg', img)
        jpg_as_text = base64.b64encode(buffer).decode('utf-8')
        
        # Prepara e invia il messaggio
        payload = {
            "image": jpg_as_text,
            "timestamp": int(time.time() * 1000)
        }
        
        print("Invio immagine al perimeter detector...")
        client.publish(PUBLISH_TOPIC, json.dumps(payload))
        
        print("Immagine inviata. Controlla il terminale del perimeter detector per i risultati.")
        print("Premi un tasto per uscire...")
        cv2.waitKey(0)
        
    except Exception as e:
        print(f"Errore: {e}")
    finally:
        client.loop_stop()
        client.disconnect()
        cv2.destroyAllWindows()
        print("Disconnesso")

if __name__ == "__main__":
    main()
