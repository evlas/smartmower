#!/usr/bin/env python3
"""
Test script per verificare la pubblicazione MQTT del state_machine
"""

import paho.mqtt.client as mqtt
import json
import time
import threading
import sys

# Configurazione MQTT
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_USERNAME = "mower"
MQTT_PASSWORD = "smart"

# Topic da monitorare
TOPICS_TO_MONITOR = [
    "smartmower/state/current",      # Stato corrente
    "smartmower/state/transitions",  # Transizioni di stato
    "smartmower/state/status",       # Eventi di stato
    "smartmower/state/commands"      # Comandi (se pubblicati)
]

# Variabili globali per il test
received_messages = {}
client = None

def on_connect(client, userdata, flags, rc):
    """Callback per la connessione MQTT"""
    if rc == 0:
        print("✅ Connesso al broker MQTT")
        # Sottoscrivi a tutti i topic di interesse
        for topic in TOPICS_TO_MONITOR:
            client.subscribe(topic)
            print(f"📡 Sottoscritto a: {topic}")
    else:
        print(f"❌ Errore di connessione MQTT: {rc}")

def on_message(client, userdata, msg):
    """Callback per i messaggi ricevuti"""
    topic = msg.topic
    try:
        payload = json.loads(msg.payload.decode())
        timestamp = time.strftime("%H:%M:%S")
        
        print(f"\n🔔 [{timestamp}] Messaggio ricevuto su: {topic}")
        print(f"📄 Payload: {json.dumps(payload, indent=2)}")
        
        # Salva il messaggio per l'analisi
        if topic not in received_messages:
            received_messages[topic] = []
        received_messages[topic].append({
            'timestamp': timestamp,
            'payload': payload
        })
        
        # Analisi specifica per tipo di messaggio
        if topic.endswith('/current'):
            analyze_current_state(payload)
        elif topic.endswith('/transitions'):
            analyze_state_transition(payload)
        elif topic.endswith('/status'):
            analyze_state_event(payload)
            
    except json.JSONDecodeError:
        print(f"⚠️  Payload non JSON su {topic}: {msg.payload.decode()}")
    except Exception as e:
        print(f"❌ Errore nel processare messaggio: {e}")

def analyze_current_state(payload):
    """Analizza i messaggi di stato corrente"""
    msg_type = payload.get('type', 'unknown')
    state = payload.get('state', 'unknown')
    uptime = payload.get('uptime', 0)
    
    print(f"   📊 Tipo: {msg_type}")
    print(f"   🏃 Stato: {state}")
    print(f"   ⏱️  Uptime: {uptime:.1f}s")

def analyze_state_transition(payload):
    """Analizza i messaggi di transizione"""
    msg_type = payload.get('type', 'unknown')
    transition = payload.get('transition', {})
    from_state = transition.get('from_state', 'unknown')
    to_state = transition.get('to_state', 'unknown')
    event = transition.get('event', 'unknown')
    
    print(f"   📊 Tipo: {msg_type}")
    print(f"   🔄 Transizione: {from_state} → {to_state}")
    print(f"   ⚡ Evento: {event}")

def analyze_state_event(payload):
    """Analizza i messaggi di eventi"""
    msg_type = payload.get('type', 'unknown')
    event_type = payload.get('event_type', 'unknown')
    event_data = payload.get('event_data', 'unknown')
    
    print(f"   📊 Tipo: {msg_type}")
    print(f"   ⚡ Evento: {event_type}")
    print(f"   📝 Dati: {event_data}")

def send_test_commands():
    """Invia comandi di test al state_machine"""
    print("\n🎮 Invio comandi di test...")
    
    test_commands = [
        "start_mowing",
        "pause", 
        "resume",
        "emergency_stop"
    ]
    
    for cmd in test_commands:
        print(f"📤 Invio comando: {cmd}")
        client.publish("smartmower/state/cmd", cmd)
        time.sleep(2)  # Attendi 2 secondi tra i comandi

def print_summary():
    """Stampa un riassunto dei messaggi ricevuti"""
    print("\n" + "="*60)
    print("📋 RIASSUNTO MESSAGGI RICEVUTI")
    print("="*60)
    
    for topic, messages in received_messages.items():
        print(f"\n📡 Topic: {topic}")
        print(f"   📊 Messaggi ricevuti: {len(messages)}")
        if messages:
            print(f"   🕐 Primo messaggio: {messages[0]['timestamp']}")
            print(f"   🕐 Ultimo messaggio: {messages[-1]['timestamp']}")

def main():
    global client
    
    print("🚀 Test pubblicazione MQTT State Machine")
    print("="*50)
    
    # Configura client MQTT
    client = mqtt.Client()
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.on_connect = on_connect
    client.on_message = on_message
    
    try:
        # Connetti al broker
        print(f"🔌 Connessione a {MQTT_BROKER}:{MQTT_PORT}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        
        # Avvia il loop MQTT in background
        client.loop_start()
        
        print("👂 In ascolto dei messaggi MQTT...")
        print("💡 Avvia il state_machine in un altro terminale per vedere i messaggi")
        print("🎮 Premi ENTER per inviare comandi di test, o 'q' per uscire")
        
        while True:
            user_input = input().strip().lower()
            if user_input == 'q':
                break
            elif user_input == '':
                send_test_commands()
            else:
                print("❓ Comandi: ENTER (test commands), 'q' (quit)")
        
    except KeyboardInterrupt:
        print("\n🛑 Interruzione da utente")
    except Exception as e:
        print(f"❌ Errore: {e}")
    finally:
        print_summary()
        client.loop_stop()
        client.disconnect()
        print("👋 Test completato")

if __name__ == "__main__":
    main()
