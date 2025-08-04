import cv2
import numpy as np
import paho.mqtt.client as mqtt
import time
import base64
import json
import os

# Configurazione
BROKER = "localhost"
PORT = 1883
USERNAME = "mower"
PASSWORD = "smart"
TOPIC = "smartmower/vision/camera"

def create_test_image():
    """Crea un'immagine di test con un bordo ben definito"""
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:] = (0, 120, 0)  # Verde
    cv2.rectangle(img, (0, 350), (640, 480), (40, 70, 140), -1)  # Bordo
    cv2.imwrite("test_img.jpg", img)
    return img

def on_connect(client, userdata, flags, rc):
    print("Connesso!" if rc == 0 else f"Errore {rc}")

client = mqtt.Client()
client.username_pw_set(USERNAME, PASSWORD)
client.on_connect = on_connect

try:
    client.connect(BROKER, PORT, 60)
    client.loop_start()
    
    img = create_test_image()
    _, buffer = cv2.imencode('.jpg', img)
    payload = {
        "image": base64.b64encode(buffer).decode('utf-8'),
        "timestamp": int(time.time() * 1000)
    }
    
    print("Invio immagine...")
    client.publish(TOPIC, json.dumps(payload))
    print("Fatto! Controlla l'output del perimeter_detector")
    
    time.sleep(2)  # Attendi la risposta
    
except Exception as e:
    print(f"Errore: {e}")
finally:
    client.loop_stop()
    client.disconnect()

