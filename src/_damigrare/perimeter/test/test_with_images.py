"""
Test del Perimeter Detector con immagini locali
"""
import os
import cv2
import numpy as np
import json
import paho.mqtt.client as mqtt
import time
import base64

# Configurazione
BROKER = "localhost"
PORT = 1883
USERNAME = "mower"
PASSWORD = "smart"
PUBLISH_TOPIC = "smartmower/vision/camera"
TEST_IMAGES_DIR = "test_images"

class PerimeterTester:
    def __init__(self):
        self.client = mqtt.Client()
        self.client.username_pw_set(USERNAME, PASSWORD)
        self.last_result = None
    
    def process_image(self, image_path):
        """Elabora un'immagine e mostra i risultati"""
        if not os.path.exists(image_path):
            print(f"File non trovato: {image_path}")
            return
            
        print(f"\nElaborazione: {os.path.basename(image_path)}")
        
        # Carica e ridimensiona l'immagine
        img = cv2.imread(image_path)
        if img is None:
            print(f"Impossibile caricare l'immagine")
            return
            
        # Converti in JPEG
        _, buffer = cv2.imencode('.jpg', img)
        jpg_as_text = base64.b64encode(buffer).decode('utf-8')
        
        # Invia al perimeter detector
        payload = {
            "image": jpg_as_text,
            "timestamp": int(time.time() * 1000)
        }
        
        self.client.publish(PUBLISH_TOPIC, json.dumps(payload))
        print("Immagine inviata al perimeter detector")
        
        # Visualizza l'immagine
        cv2.imshow("Test Image", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def create_test_images():
    """Crea immagini di test"""
    os.makedirs(TEST_IMAGES_DIR, exist_ok=True)
    
    # Bordo orizzontale
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:] = (0, 100, 0)  # Sfondo verde
    cv2.rectangle(img, (0, 400), (640, 480), (30, 60, 120), -1)  # Bordo
    cv2.imwrite(f"{TEST_IMAGES_DIR}/border_horizontal.jpg", img)
    
    # Bordo verticale
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:] = (0, 100, 0)
    cv2.rectangle(img, (500, 0), (640, 480), (30, 60, 120), -1)
    cv2.imwrite(f"{TEST_IMAGES_DIR}/border_vertical.jpg", img)

def main():
    # Crea immagini di test
    create_test_images()
    
    # Inizializza il tester
    tester = PerimeterTester()
    
    try:
        # Connetti al broker
        print(f"Connessione a {BROKER}:{PORT}...")
        tester.client.connect(BROKER, PORT, 60)
        
        # Elabora le immagini di test
        for img in os.listdir(TEST_IMAGES_DIR):
            if img.lower().endswith(('.png', '.jpg', '.jpeg')):
                tester.process_image(f"{TEST_IMAGES_DIR}/{img}")
                
    except Exception as e:
        print(f"Errore: {e}")
    finally:
        tester.client.disconnect()
        print("Disconnesso")

if __name__ == "__main__":
    main()
