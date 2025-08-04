#!/usr/bin/env python3
"""
Test di stress per la pubblicazione MQTT della state machine
Simula un carico intenso di messaggi per verificare la stabilità
"""

import paho.mqtt.client as mqtt
import json
import time
import threading
import random
from datetime import datetime

# Configurazione MQTT
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_USERNAME = "mower"
MQTT_PASSWORD = "smart"

class MQTTStressTest:
    def __init__(self):
        self.client = mqtt.Client()
        self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        self.message_count = 0
        self.start_time = time.time()
        self.running = True
        
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("✅ Connesso al broker MQTT")
            # Sottoscrivi ai topic della state machine
            client.subscribe("smartmower/state/+")
            client.subscribe("smartmower/commands/+")
        else:
            print(f"❌ Connessione fallita: {rc}")
    
    def on_message(self, client, userdata, msg):
        self.message_count += 1
        topic = msg.topic
        timestamp = datetime.now().strftime("%H:%M:%S")
        try:
            payload = json.loads(msg.payload.decode())
            print(f"[{timestamp}] {topic}: {payload.get('type', 'unknown')}")
        except:
            print(f"[{timestamp}] {topic}: Raw message")
    
    def simulate_high_frequency_data(self):
        """Simula dati ad alta frequenza come fusion sensor"""
        while self.running:
            # Simula fusion data (10Hz)
            fusion_data = {
                "type": "fusion_data",
                "timestamp": int(time.time() * 1000),
                "position": {
                    "x": random.uniform(-10, 10),
                    "y": random.uniform(-10, 10),
                    "z": 0.0
                },
                "velocity": {
                    "vx": random.uniform(-2, 2),
                    "vy": random.uniform(-2, 2),
                    "vz": 0.0,
                    "speed": random.uniform(0, 2)
                },
                "orientation": {
                    "roll": random.uniform(-0.1, 0.1),
                    "pitch": random.uniform(-0.1, 0.1),
                    "yaw": random.uniform(0, 6.28)
                },
                "uncertainty": {
                    "position_x": random.uniform(0.01, 0.1),
                    "position_y": random.uniform(0.01, 0.1),
                    "velocity_x": random.uniform(0.01, 0.1),
                    "velocity_y": random.uniform(0.01, 0.1),
                    "yaw": random.uniform(0.01, 0.1)
                }
            }
            
            self.client.publish("smartmower/fusion/data", json.dumps(fusion_data))
            time.sleep(0.1)  # 10Hz
    
    def simulate_slam_data(self):
        """Simula dati SLAM"""
        while self.running:
            slam_data = {
                "type": "slam_pose",
                "timestamp": int(time.time()),
                "pose": {
                    "x": random.uniform(-5, 5),
                    "y": random.uniform(-5, 5),
                    "theta": random.uniform(0, 6.28),
                    "confidence": random.uniform(0.8, 1.0)
                },
                "covariance": [
                    random.uniform(0.001, 0.01), 0.0, 0.0,
                    0.0, random.uniform(0.001, 0.01), 0.0
                ]
            }
            
            self.client.publish("smartmower/slam/pose", json.dumps(slam_data))
            time.sleep(0.5)  # 2Hz
    
    def simulate_heartbeats(self):
        """Simula heartbeat dei componenti"""
        while self.running:
            # Pico heartbeat
            pico_heartbeat = {
                "type": "pico_heartbeat",
                "status": "running",
                "timestamp": int(time.time()),
                "battery_level": random.uniform(20, 100),
                "system_status": "operational"
            }
            self.client.publish("smartmower/pico/heartbeat", json.dumps(pico_heartbeat))
            
            # GPS heartbeat
            gps_heartbeat = {
                "type": "gps_heartbeat",
                "status": "running",
                "timestamp": int(time.time()),
                "satellites": random.randint(8, 15),
                "fix_quality": "3D"
            }
            self.client.publish("smartmower/gps/heartbeat", json.dumps(gps_heartbeat))
            
            time.sleep(2.0)  # 0.5Hz
    
    def send_test_commands(self):
        """Invia comandi di test periodici"""
        commands = [
            "start_mowing",
            "stop_mowing", 
            "return_to_dock",
            "emergency_stop",
            "resume_operation"
        ]
        
        while self.running:
            time.sleep(10)  # Ogni 10 secondi
            if self.running:
                cmd = random.choice(commands)
                print(f"🔧 Invio comando: {cmd}")
                self.client.publish("smartmower/commands/state", cmd)
    
    def run_stress_test(self, duration_minutes=5):
        """Esegue il test di stress per la durata specificata"""
        print(f"🚀 Avvio test di stress MQTT per {duration_minutes} minuti...")
        
        # Connetti al broker
        self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.client.loop_start()
        
        # Avvia thread per simulare dati
        threads = [
            threading.Thread(target=self.simulate_high_frequency_data, daemon=True),
            threading.Thread(target=self.simulate_slam_data, daemon=True),
            threading.Thread(target=self.simulate_heartbeats, daemon=True),
            threading.Thread(target=self.send_test_commands, daemon=True)
        ]
        
        for thread in threads:
            thread.start()
        
        # Esegui per la durata specificata
        try:
            time.sleep(duration_minutes * 60)
        except KeyboardInterrupt:
            print("\n⏹️  Test interrotto dall'utente")
        
        self.running = False
        
        # Statistiche finali
        elapsed = time.time() - self.start_time
        print(f"\n📊 Test completato:")
        print(f"   Durata: {elapsed:.1f} secondi")
        print(f"   Messaggi ricevuti: {self.message_count}")
        print(f"   Frequenza media: {self.message_count/elapsed:.1f} msg/sec")
        
        self.client.loop_stop()
        self.client.disconnect()

if __name__ == "__main__":
    import sys
    
    duration = 3  # Default 3 minuti
    if len(sys.argv) > 1:
        try:
            duration = int(sys.argv[1])
        except ValueError:
            print("Uso: python3 test_mqtt_stress.py [durata_minuti]")
            sys.exit(1)
    
    test = MQTTStressTest()
    test.run_stress_test(duration)
