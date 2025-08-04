#!/usr/bin/env python3
"""
Monitor MQTT per verificare la pubblicazione del perimeter detector
"""

import paho.mqtt.client as mqtt
import json
import time
from datetime import datetime

# Configurazione MQTT
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_USERNAME = "mower"
MQTT_PASSWORD = "smart"

class PerimeterMonitor:
    def __init__(self):
        self.client = mqtt.Client()
        self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        self.message_count = 0
        self.start_time = time.time()
        
        print(f"🔍 Perimeter Monitor inizializzato")
        print(f"   MQTT: {MQTT_BROKER}:{MQTT_PORT} (user: {MQTT_USERNAME})")
    
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("✅ Connesso al broker MQTT")
            # Sottoscrivi ai topic del perimeter detector
            topics = [
                "smartmower/vision/perimeter",
                "smartmower/vision/perimeter/status",
                "smartmower/vision/perimeter/config",
                "smartmower/vision/+",  # Tutti i topic vision
                "+/+/perimeter"         # Pattern generico per perimeter
            ]
            
            for topic in topics:
                client.subscribe(topic)
                print(f"📡 Sottoscritto a: {topic}")
        else:
            print(f"❌ Connessione fallita: {rc}")
    
    def on_message(self, client, userdata, msg):
        self.message_count += 1
        topic = msg.topic
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        try:
            payload = json.loads(msg.payload.decode())
            msg_type = payload.get('type', 'unknown')
            
            print(f"[{timestamp}] 📨 {topic}")
            print(f"    Type: {msg_type}")
            
            # Mostra dettagli specifici per perimeter detection
            if 'perimeter' in msg_type.lower():
                if 'detection' in payload:
                    detection = payload['detection']
                    print(f"    Detection: {detection}")
                
                if 'boundary_distance' in payload:
                    distance = payload['boundary_distance']
                    print(f"    Boundary Distance: {distance}")
                
                if 'confidence' in payload:
                    confidence = payload['confidence']
                    print(f"    Confidence: {confidence}")
            
            # Mostra payload completo se piccolo
            if len(str(payload)) < 200:
                print(f"    Payload: {payload}")
            else:
                print(f"    Payload size: {len(msg.payload)} bytes")
                
        except json.JSONDecodeError:
            print(f"[{timestamp}] 📨 {topic} (non-JSON)")
            print(f"    Raw: {msg.payload.decode()[:100]}...")
        except Exception as e:
            print(f"[{timestamp}] 📨 {topic} (error: {e})")
        
        print()
    
    def monitor(self, duration_minutes=5):
        """Monitora i messaggi MQTT per la durata specificata"""
        print(f"🎯 Avvio monitoraggio perimeter per {duration_minutes} minuti...")
        print("   Aspettando messaggi MQTT dal perimeter detector...")
        
        # Connetti al broker
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.client.loop_start()
        except Exception as e:
            print(f"❌ Errore connessione MQTT: {e}")
            return
        
        start_time = time.time()
        end_time = start_time + (duration_minutes * 60)
        
        try:
            while time.time() < end_time:
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n⏹️  Monitoraggio interrotto dall'utente")
        
        # Statistiche finali
        elapsed = time.time() - start_time
        print(f"\n📊 Monitoraggio completato:")
        print(f"   Durata: {elapsed:.1f} secondi")
        print(f"   Messaggi ricevuti: {self.message_count}")
        if self.message_count > 0:
            print(f"   Frequenza media: {self.message_count/elapsed:.2f} msg/sec")
        else:
            print("   ⚠️  Nessun messaggio ricevuto dal perimeter detector")
        
        self.client.loop_stop()
        self.client.disconnect()

if __name__ == "__main__":
    import sys
    
    duration = 2  # Default 2 minuti
    if len(sys.argv) > 1:
        try:
            duration = int(sys.argv[1])
        except ValueError:
            print("Uso: python3 perimeter_monitor.py [durata_minuti]")
            sys.exit(1)
    
    monitor = PerimeterMonitor()
    monitor.monitor(duration)
