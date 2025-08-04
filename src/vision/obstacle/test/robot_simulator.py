#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import numpy as np
import cv2
import time
import base64
import json
import math
from datetime import datetime
import sys
import os

# Add parent directory to path to import config_utils
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
from config_utils import get_mqtt_credentials_safe

class RobotSimulator:
    def __init__(self):
        # MQTT Configuration
        broker_host, username, password, port = get_mqtt_credentials_safe()
        print(f"Using MQTT config: {broker_host}:{port} (user: {username})")

        # Parametri MQTT
        self.mqtt_broker = broker_host
        self.mqtt_port = port
        self.mqtt_username = username
        self.mqtt_password = password
        self.camera_topic = "smartmower/vision/camera"
        self.velocity_topic = "smartmower/fusion/data"
        
        # Parametri simulazione
        self.frame_width = 640
        self.frame_height = 480
        self.obstacle_width = 100  # Larghezza ostacolo in pixel
        self.initial_distance = 5.0  # Distanza iniziale in metri
        self.velocity = 0.1  # m/s
        self.fps = 10
        self.simulation_time = 100  # secondi
        
        # Stato simulazione
        self.distance = self.initial_distance
        self.obstacle_x = self.frame_width // 2
        self.obstacle_y = self.frame_height // 2
        
        # Inizializza MQTT
        self.client = mqtt.Client()
        self.client.username_pw_set(self.mqtt_username, self.mqtt_password)
        
    def generate_frame(self):
        """Genera un frame con un ostacolo alla distanza corrente"""
        # Crea un'immagine nera
        frame = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)
        
        # Calcola la dimensione dell'ostacolo in base alla distanza
        # Più è vicino, più grande appare
        obstacle_size = int((1.0 / self.distance) * 300)
        obstacle_size = max(10, min(200, obstacle_size))  # Limita la dimensione
        
        # Disegna l'ostacolo (rettangolo rosso)
        x1 = self.obstacle_x - obstacle_size // 2
        y1 = self.obstacle_y - obstacle_size // 2
        x2 = x1 + obstacle_size
        y2 = y1 + obstacle_size
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), -1)
        
        # Aggiungi testo con la distanza
        cv2.putText(frame, f"Distanza: {self.distance:.2f}m", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return frame
    
    def encode_image(self, frame):
        """Codifica l'immagine in base64"""
        _, buffer = cv2.imencode('.jpg', frame)
        return base64.b64encode(buffer).decode('utf-8')
    
    def publish_frame(self, frame):
        """Pubblica il frame su MQTT"""
        # Codifica l'immagine
        img_str = self.encode_image(frame)
        
        # Crea il messaggio
        message = {
            "timestamp": datetime.now().isoformat(),
            "image": img_str
        }
        
        # Pubblica
        self.client.publish(self.camera_topic, json.dumps(message))
    
    def publish_velocity(self):
        """Pubblica i dati di fusione nel nuovo formato espanso"""
        import time
        
        # Simula movimento del robot
        current_time = time.time()
        if not hasattr(self, '_last_time'):
            self._last_time = current_time
            self._position_x = 0.0
            self._position_y = 0.0
            self._yaw = 0.0
        
        dt = current_time - self._last_time
        self._last_time = current_time
        
        # Aggiorna posizione simulata
        self._position_x += self.velocity * dt * 0.8  # Componente X
        self._position_y += self.velocity * dt * 0.6  # Componente Y
        self._yaw += 0.1 * dt  # Rotazione lenta
        
        # Crea messaggio nel nuovo formato fusion_data
        message = {
            "type": "fusion_data",
            "timestamp": int(current_time * 1000),  # Timestamp in millisecondi
            "position": {
                "x": self._position_x,
                "y": self._position_y,
                "z": 0.1  # Altezza simulata
            },
            "velocity": {
                "vx": self.velocity * 0.8,  # Componente X della velocità
                "vy": self.velocity * 0.6,  # Componente Y della velocità
                "vz": 0.0,  # Componente Z della velocità
                "speed": self.velocity  # Velocità totale
            },
            "orientation": {
                "roll": 0.05,  # Roll simulato
                "pitch": 0.02,  # Pitch simulato
                "yaw": self._yaw  # Yaw simulato
            },
            "uncertainty": {
                "position_x": 0.1,  # Incertezza posizione X
                "position_y": 0.1,  # Incertezza posizione Y
                "velocity_x": 0.05,  # Incertezza velocità X
                "velocity_y": 0.05,  # Incertezza velocità Y
                "yaw": 0.02  # Incertezza yaw
            }
        }
        
        self.client.publish(self.velocity_topic, json.dumps(message))
    
    def run_simulation(self):
        """Esegue la simulazione"""
        print(f"Avvio simulazione per {self.simulation_time} secondi...")
        
        # Connessione MQTT
        self.client.connect(self.mqtt_broker, self.mqtt_port)
        self.client.loop_start()
        
        start_time = time.time()
        last_frame_time = 0
        
        while (time.time() - start_time) < self.simulation_time:
            current_time = time.time()
            
            # Calcola il tempo trascorso dall'ultimo frame
            dt = current_time - last_frame_time if last_frame_time > 0 else 1.0/self.fps
            
            # Aggiorna la distanza in base alla velocità
            self.distance = max(0.1, self.distance - (self.velocity * dt))
            
            # Genera e pubblica il frame
            frame = self.generate_frame()
            self.publish_frame(frame)
            
            # Pubblica la velocità
            self.publish_velocity()
            
            # Mostra l'anteprima
            cv2.imshow("Robot Simulator", frame)
            
            # Attendi il prossimo frame
            last_frame_time = current_time
            time_to_wait = max(0, (1.0 / self.fps) - (time.time() - current_time))
            
            # Interrompi se viene premuto 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
            time.sleep(time_to_wait)
        
        # Pulizia
        cv2.destroyAllWindows()
        self.client.loop_stop()
        print("Simulazione completata")

if __name__ == "__main__":
    simulator = RobotSimulator()
    simulator.run_simulation()
