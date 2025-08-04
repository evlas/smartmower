#!/usr/bin/env python3
"""
Monitor completo per tutti i moduli di vision (grass, perimeter, obstacle)
Verifica la pubblicazione MQTT di tutti i detector
"""

import paho.mqtt.client as mqtt
import json
import time
from datetime import datetime
import threading

# Configurazione MQTT
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_USERNAME = "mower"
MQTT_PASSWORD = "smart"

class VisionMonitor:
    def __init__(self):
        self.client = mqtt.Client()
        self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        self.message_counts = {
            'grass': 0,
            'perimeter': 0,
            'obstacle': 0,
            'camera': 0,
            'other': 0
        }
        
        self.last_messages = {
            'grass': None,
            'perimeter': None,
            'obstacle': None,
            'camera': None
        }
        
        self.start_time = time.time()
        
        print(f"🔍 Vision Monitor inizializzato")
        print(f"   MQTT: {MQTT_BROKER}:{MQTT_PORT} (user: {MQTT_USERNAME})")
    
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("✅ Connesso al broker MQTT")
            # Sottoscrivi a tutti i topic vision
            topics = [
                "smartmower/vision/+",           # Tutti i topic vision principali
                "smartmower/vision/+/status",    # Status di tutti i moduli
                "smartmower/vision/+/config",    # Config di tutti i moduli
                "smartmower/+/+",                # Pattern generico
                "+/vision/+",                    # Pattern alternativo
            ]
            
            for topic in topics:
                client.subscribe(topic)
                print(f"📡 Sottoscritto a: {topic}")
        else:
            print(f"❌ Connessione fallita: {rc}")
    
    def on_message(self, client, userdata, msg):
        topic = msg.topic
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        # Classifica il messaggio per tipo
        message_type = 'other'
        if 'grass' in topic:
            message_type = 'grass'
        elif 'perimeter' in topic:
            message_type = 'perimeter'
        elif 'obstacle' in topic:
            message_type = 'obstacle'
        elif 'camera' in topic:
            message_type = 'camera'
        
        self.message_counts[message_type] += 1
        
        try:
            payload = json.loads(msg.payload.decode())
            msg_payload_type = payload.get('type', 'unknown')
            
            print(f"[{timestamp}] 📨 {topic}")
            print(f"    Type: {msg_payload_type}")
            print(f"    Category: {message_type.upper()}")
            
            # Salva l'ultimo messaggio per categoria
            self.last_messages[message_type] = {
                'timestamp': timestamp,
                'topic': topic,
                'payload': payload
            }
            
            # Mostra dettagli specifici per ogni tipo
            if message_type == 'grass':
                self._show_grass_details(payload)
            elif message_type == 'perimeter':
                self._show_perimeter_details(payload)
            elif message_type == 'obstacle':
                self._show_obstacle_details(payload)
            elif message_type == 'camera':
                self._show_camera_details(payload)
            
            # Mostra payload se piccolo
            if len(str(payload)) < 300:
                print(f"    Payload: {payload}")
            else:
                print(f"    Payload size: {len(msg.payload)} bytes")
                
        except json.JSONDecodeError:
            print(f"[{timestamp}] 📨 {topic} (non-JSON)")
            print(f"    Raw: {msg.payload.decode()[:100]}...")
            self.message_counts[message_type] += 1
        except Exception as e:
            print(f"[{timestamp}] 📨 {topic} (error: {e})")
            self.message_counts[message_type] += 1
        
        print()
    
    def _show_grass_details(self, payload):
        """Mostra dettagli specifici per grass detection"""
        if 'height' in payload:
            print(f"    🌱 Grass Height: {payload['height']} cm")
        if 'coverage' in payload:
            print(f"    🌱 Coverage: {payload['coverage']}%")
        if 'pixels' in payload:
            print(f"    🌱 Pixels: {payload['pixels']}")
    
    def _show_perimeter_details(self, payload):
        """Mostra dettagli specifici per perimeter detection"""
        if 'distance' in payload:
            print(f"    🚧 Distance: {payload['distance']} m")
        if 'warning' in payload:
            print(f"    🚧 Warning: {payload['warning']}")
        if 'alert' in payload:
            print(f"    🚧 Alert: {payload['alert']}")
        if 'contours' in payload:
            print(f"    🚧 Contours: {payload['contours']}")
    
    def _show_obstacle_details(self, payload):
        """Mostra dettagli specifici per obstacle detection"""
        if 'obstacles' in payload:
            obstacles = payload['obstacles']
            print(f"    🚫 Obstacles detected: {len(obstacles)}")
            for i, obs in enumerate(obstacles[:3]):  # Mostra solo i primi 3
                if 'distance' in obs:
                    print(f"    🚫 Obstacle {i+1}: {obs['distance']} m")
    
    def _show_camera_details(self, payload):
        """Mostra dettagli specifici per camera data"""
        if 'width' in payload and 'height' in payload:
            print(f"    📷 Resolution: {payload['width']}x{payload['height']}")
        if 'frame_id' in payload:
            print(f"    📷 Frame ID: {payload['frame_id']}")
        if 'format' in payload:
            print(f"    📷 Format: {payload['format']}")
    
    def show_statistics(self):
        """Mostra statistiche correnti"""
        elapsed = time.time() - self.start_time
        print(f"\n📊 Statistiche Vision Monitor:")
        print(f"   Durata: {elapsed:.1f} secondi")
        print(f"   📷 Camera: {self.message_counts['camera']} messaggi")
        print(f"   🌱 Grass: {self.message_counts['grass']} messaggi")
        print(f"   🚧 Perimeter: {self.message_counts['perimeter']} messaggi")
        print(f"   🚫 Obstacle: {self.message_counts['obstacle']} messaggi")
        print(f"   📨 Altri: {self.message_counts['other']} messaggi")
        print(f"   📈 Totale: {sum(self.message_counts.values())} messaggi")
        
        # Mostra ultimo messaggio per categoria
        print(f"\n📋 Ultimi messaggi ricevuti:")
        for category, msg in self.last_messages.items():
            if msg:
                print(f"   {category.upper()}: {msg['timestamp']} - {msg['topic']}")
            else:
                print(f"   {category.upper()}: Nessun messaggio ricevuto")
    
    def monitor(self, duration_minutes=3):
        """Monitora i messaggi MQTT per la durata specificata"""
        print(f"🎯 Avvio monitoraggio vision completo per {duration_minutes} minuti...")
        print("   Aspettando messaggi MQTT da tutti i moduli vision...")
        
        # Connetti al broker
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.client.loop_start()
        except Exception as e:
            print(f"❌ Errore connessione MQTT: {e}")
            return
        
        start_time = time.time()
        end_time = start_time + (duration_minutes * 60)
        
        # Mostra statistiche ogni 30 secondi
        last_stats = start_time
        
        try:
            while time.time() < end_time:
                current_time = time.time()
                
                # Mostra statistiche periodiche
                if current_time - last_stats >= 30:
                    self.show_statistics()
                    last_stats = current_time
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n⏹️  Monitoraggio interrotto dall'utente")
        
        # Statistiche finali
        self.show_statistics()
        
        # Analisi finale
        print(f"\n🔍 Analisi finale:")
        if self.message_counts['grass'] > 0:
            print("   ✅ Grass Detector: ATTIVO e pubblica")
        else:
            print("   ❌ Grass Detector: NON pubblica")
            
        if self.message_counts['perimeter'] > 0:
            print("   ✅ Perimeter Detector: ATTIVO e pubblica")
        else:
            print("   ❌ Perimeter Detector: NON pubblica")
            
        if self.message_counts['obstacle'] > 0:
            print("   ✅ Obstacle Detector: ATTIVO e pubblica")
        else:
            print("   ❌ Obstacle Detector: NON pubblica")
            
        if self.message_counts['camera'] > 0:
            print("   ✅ Camera: ATTIVA e pubblica")
        else:
            print("   ❌ Camera: NON pubblica")
        
        self.client.loop_stop()
        self.client.disconnect()

def run_comprehensive_test():
    """Esegue un test completo con camera simulator e monitor"""
    print("🚀 Avvio test completo vision system...")
    
    # Avvia il monitor
    monitor = VisionMonitor()
    
    # Avvia il monitor in un thread separato
    monitor_thread = threading.Thread(
        target=monitor.monitor, 
        args=(2,),  # 2 minuti di monitoraggio
        daemon=True
    )
    monitor_thread.start()
    
    # Aspetta un po' prima di avviare il simulatore
    time.sleep(2)
    
    # Avvia il simulatore camera
    import subprocess
    simulator_process = subprocess.Popen([
        'python3', 'camera_simulator.py', 
        '--fps', '1.0', 
        '--duration', '1'
    ])
    
    # Aspetta che il test finisca
    monitor_thread.join()
    
    # Termina il simulatore se ancora in esecuzione
    try:
        simulator_process.terminate()
        simulator_process.wait(timeout=5)
    except:
        pass
    
    print("✅ Test completo terminato")

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == '--comprehensive':
        run_comprehensive_test()
    else:
        duration = 2  # Default 2 minuti
        if len(sys.argv) > 1:
            try:
                duration = int(sys.argv[1])
            except ValueError:
                print("Uso: python3 vision_monitor.py [durata_minuti] | --comprehensive")
                sys.exit(1)
        
        monitor = VisionMonitor()
        monitor.monitor(duration)
