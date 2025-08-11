#!/usr/bin/env python3
"""LCD Interface for Smart Mower"""

import time
import json
import threading
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO

# Hardware config
I2C_ADDR = 0x27
BUTTONS = {'UP': 17, 'DOWN': 22, 'LEFT': 5, 'RIGHT': 6, 'SELECT': 13, 'EMERGENCY': 19}
BUZZER_PIN = 26

class LCDInterface:
    def __init__(self, config_path="/opt/smartmower/etc/config/robot_config.json"):
        self.config = self._load_config(config_path)
        self.state = "INIT"
        self.battery = 0
        self.info = {}
        self.menu = self._init_menu()
        self._setup_hw()
        self._setup_mqtt()
        
    def _load_config(self, path):
        try:
            with open(path) as f:
                return json.load(f)
        except:
            return {}
            
    def _setup_hw(self):
        GPIO.setmode(GPIO.BCM)
        for pin in BUTTONS.values():
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(pin, GPIO.FALLING, 
                               callback=self._button_press,
                               bouncetime=200)
        GPIO.setup(BUZZER_PIN, GPIO.OUT)
        self.buzzer = GPIO.PWM(BUZZER_PIN, 1000)
        self.buzzer.start(0)
        
    def _setup_mqtt(self):
        self.mqtt = mqtt.Client()
        cfg = self.config.get('system', {}).get('communication', {})
        self.mqtt.username_pw_set(cfg.get('mqtt_username', 'mower'),
                                cfg.get('mqtt_password', 'smart'))
        self.mqtt.on_connect = self._on_connect
        self.mqtt.on_message = self._on_message
        self.mqtt.connect(cfg.get('mqtt_broker_host', 'localhost'),
                         cfg.get('mqtt_broker_port', 1883))
        threading.Thread(target=self.mqtt.loop_forever, daemon=True).start()
        
    def _on_connect(self, client, userdata, flags, rc):
        self.mqtt.subscribe([("robot/state", 0), ("robot/battery", 0), ("robot/info", 0)])
        
    def _on_message(self, client, userdata, msg):
        try:
            if msg.topic == "robot/state":
                self.state = msg.payload.decode()
            elif msg.topic == "robot/battery":
                self.battery = float(msg.payload)
            elif msg.topic == "robot/info":
                self.info = json.loads(msg.payload)
        except Exception as e:
            print(f"MQTT error: {e}")
            
    def _button_press(self, channel):
        if channel == BUTTONS['EMERGENCY']:
            self._emergency_stop()
            return
            
        # Handle other buttons
        for name, pin in BUTTONS.items():
            if pin == channel and name != 'EMERGENCY':
                self._handle_button(name)
                break
                
    def _emergency_stop(self):
        self.mqtt.publish("robot/command", "EMERGENCY_STOP")
        self._beep(2000, 1000, 100)  # Loud emergency beep
        
    def _beep(self, freq, duration, volume=50):
        self.buzzer.ChangeFrequency(freq)
        self.buzzer.ChangeDutyCycle(volume)
        time.sleep(duration/1000)
        self.buzzer.ChangeDutyCycle(0)
        
    def _init_menu(self):
        return {
            'main': ["Start Mowing", "Stop Mowing", "Settings", "Back"],
            'settings': ["Buzzer Volume", "Back"]
        }
        
    def _handle_button(self, button):
        # Implement menu navigation and actions
        self._beep(1000, 100)  # Button feedback
        
    def update_display(self):
        # Update LCD with current state/menu
        line1 = f"{self.state} {self.battery}%"
        line2 = "Menu: " + " ".join(self.menu['main'][:2])
        print(f"LCD: {line1}\n{line2}")  # Replace with actual LCD update

def main():
    interface = LCDInterface()
    try:
        while True:
            interface.update_display()
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
