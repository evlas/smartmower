#!/usr/bin/env python3
"""
Simulatore di camera per testare il grass detector
Pubblica immagini fittizie in movimento su MQTT
"""

import paho.mqtt.client as mqtt
import json
import time
import base64
import numpy as np
from PIL import Image, ImageDraw
import io
import threading
import sys
import os

# Aggiungi il path per il modulo di configurazione
sys.path.append('/home/vito/smartmower/src/vision/grass')
try:
    from config_utils import load_mqtt_config
    mqtt_config = load_mqtt_config()
    MQTT_BROKER = mqtt_config['host']
    MQTT_PORT = mqtt_config['port']
    MQTT_USERNAME = mqtt_config['username']
    MQTT_PASSWORD = mqtt_config['password']
except:
    # Fallback ai valori di default
    MQTT_BROKER = "localhost"
    MQTT_PORT = 1883
    MQTT_USERNAME = "mower"
    MQTT_PASSWORD = "smart"

class CameraSimulator:
    def __init__(self):
        self.client = mqtt.Client()
        self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        # Parametri simulazione
        self.width = 640
        self.height = 480
        self.frame_count = 0
        self.running = True
        
        # Posizione oggetti in movimento
        self.grass_patches = [
            {"x": 150, "y": 200, "dx": 1, "dy": 0.5, "size": 60, "density": 0.8},
            {"x": 350, "y": 150, "dx": -0.5, "dy": 1, "size": 45, "density": 0.6},
            {"x": 480, "y": 320, "dx": 0.8, "dy": -0.8, "size": 55, "density": 0.9}
        ]
        
        # Ostacoli fissi e mobili
        self.obstacles = [
            {"x": 200, "y": 100, "dx": 0.3, "dy": 0.2, "width": 40, "height": 30, "type": "rock"},
            {"x": 450, "y": 200, "dx": -0.2, "dy": 0.4, "width": 35, "height": 35, "type": "tree_stump"},
            {"x": 100, "y": 350, "dx": 0.1, "dy": -0.3, "width": 50, "height": 25, "type": "debris"}
        ]
        
        # Perimetro del prato (bordi fissi)
        self.perimeter_lines = [
            {"start": (50, 50), "end": (590, 50), "thickness": 8, "type": "fence"},      # Top
            {"start": (590, 50), "end": (590, 430), "thickness": 8, "type": "fence"},    # Right
            {"start": (590, 430), "end": (50, 430), "thickness": 8, "type": "fence"},    # Bottom
            {"start": (50, 430), "end": (50, 50), "thickness": 8, "type": "fence"}       # Left
        ]
        
        # Elementi perimetrali aggiuntivi
        self.perimeter_features = [
            {"x": 300, "y": 45, "width": 60, "height": 15, "type": "gate"},
            {"x": 585, "y": 240, "width": 15, "height": 40, "type": "post"}
        ]
        
        print(f"🎥 Camera Simulator inizializzato")
        print(f"   Risoluzione: {self.width}x{self.height}")
        print(f"   MQTT: {MQTT_BROKER}:{MQTT_PORT} (user: {MQTT_USERNAME})")
    
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("✅ Connesso al broker MQTT")
        else:
            print(f"❌ Connessione fallita: {rc}")
    
    def on_message(self, client, userdata, msg):
        # Non ci aspettiamo messaggi, ma gestiamo per completezza
        pass
    
    def create_synthetic_image(self):
        """Crea un'immagine sintetica completa con perimetri, ostacoli e erba"""
        # Crea immagine base (terreno marrone/verde misto)
        img = Image.new('RGB', (self.width, self.height), color=(85, 107, 47))  # Verde oliva scuro
        draw = ImageDraw.Draw(img)
        
        # Aggiungi texture di terreno realistico
        for i in range(0, self.width, 15):
            for j in range(0, self.height, 15):
                # Variazione casuale del colore del terreno (mix verde-marrone)
                grass_factor = np.random.random()
                if grass_factor > 0.3:  # 70% erba di base
                    base_color = (85, 107, 47)  # Verde oliva
                else:  # 30% terreno nudo
                    base_color = (139, 69, 19)  # Marrone
                
                variation = np.random.randint(-15, 15)
                color = (
                    max(0, min(255, base_color[0] + variation)),
                    max(0, min(255, base_color[1] + variation)),
                    max(0, min(255, base_color[2] + variation))
                )
                draw.rectangle([i, j, i+8, j+8], fill=color)
        
        # 1. DISEGNA PERIMETRO (prima di tutto per essere sullo sfondo)
        self._draw_perimeter(draw)
        
        # 2. DISEGNA OSTACOLI
        self._draw_obstacles(draw)
        
        # 3. DISEGNA PATCH DI ERBA (sopra tutto il resto)
        self._draw_grass_patches(draw)
        
        # Aggiungi informazioni di debug
        self._draw_debug_info(draw)
        
        self.frame_count += 1
        return img
    
    def _draw_perimeter(self, draw):
        """Disegna il perimetro del prato (recinzioni, bordi)"""
        # Disegna linee perimetrali principali
        for line in self.perimeter_lines:
            if line["type"] == "fence":
                # Recinzione marrone scura
                fence_color = (101, 67, 33)  # Marrone scuro
                draw.line([line["start"], line["end"]], fill=fence_color, width=line["thickness"])
                
                # Aggiungi texture della recinzione (linee verticali)
                start_x, start_y = line["start"]
                end_x, end_y = line["end"]
                
                if start_x == end_x:  # Linea verticale
                    for y in range(min(start_y, end_y), max(start_y, end_y), 20):
                        draw.line([(start_x-2, y), (start_x+2, y)], fill=(80, 50, 25), width=2)
                else:  # Linea orizzontale
                    for x in range(min(start_x, end_x), max(start_x, end_x), 20):
                        draw.line([(x, start_y-2), (x, start_y+2)], fill=(80, 50, 25), width=2)
        
        # Disegna elementi perimetrali speciali
        for feature in self.perimeter_features:
            if feature["type"] == "gate":
                # Cancello grigio metallico
                gate_color = (128, 128, 128)
                draw.rectangle([
                    feature["x"], feature["y"],
                    feature["x"] + feature["width"], feature["y"] + feature["height"]
                ], fill=gate_color, outline=(64, 64, 64), width=2)
                
            elif feature["type"] == "post":
                # Palo di recinzione
                post_color = (101, 67, 33)
                draw.rectangle([
                    feature["x"], feature["y"],
                    feature["x"] + feature["width"], feature["y"] + feature["height"]
                ], fill=post_color, outline=(80, 50, 25), width=1)
    
    def _draw_obstacles(self, draw):
        """Disegna ostacoli mobili e fissi"""
        for obstacle in self.obstacles:
            # Aggiorna posizione (movimento lento)
            obstacle["x"] += obstacle["dx"]
            obstacle["y"] += obstacle["dy"]
            
            # Rimbalza sui bordi (con margine per il perimetro)
            margin = 60
            if obstacle["x"] <= margin or obstacle["x"] >= self.width - margin - obstacle["width"]:
                obstacle["dx"] *= -1
            if obstacle["y"] <= margin or obstacle["y"] >= self.height - margin - obstacle["height"]:
                obstacle["dy"] *= -1
            
            # Disegna diversi tipi di ostacoli
            if obstacle["type"] == "rock":
                # Roccia grigia
                rock_color = (105, 105, 105)
                draw.ellipse([
                    obstacle["x"], obstacle["y"],
                    obstacle["x"] + obstacle["width"], obstacle["y"] + obstacle["height"]
                ], fill=rock_color, outline=(70, 70, 70), width=2)
                
                # Aggiungi texture rocciosa
                for i in range(3):
                    spot_x = obstacle["x"] + np.random.randint(5, obstacle["width"]-5)
                    spot_y = obstacle["y"] + np.random.randint(5, obstacle["height"]-5)
                    draw.ellipse([spot_x-2, spot_y-2, spot_x+2, spot_y+2], fill=(85, 85, 85))
                    
            elif obstacle["type"] == "tree_stump":
                # Ceppo di albero marrone
                stump_color = (139, 69, 19)
                draw.ellipse([
                    obstacle["x"], obstacle["y"],
                    obstacle["x"] + obstacle["width"], obstacle["y"] + obstacle["height"]
                ], fill=stump_color, outline=(101, 50, 15), width=2)
                
                # Anelli del legno
                center_x = obstacle["x"] + obstacle["width"] // 2
                center_y = obstacle["y"] + obstacle["height"] // 2
                for r in range(5, min(obstacle["width"], obstacle["height"]) // 2, 5):
                    draw.ellipse([center_x-r, center_y-r, center_x+r, center_y+r], 
                               outline=(101, 50, 15), width=1)
                    
            elif obstacle["type"] == "debris":
                # Detriti/rami
                debris_color = (160, 82, 45)
                draw.rectangle([
                    obstacle["x"], obstacle["y"],
                    obstacle["x"] + obstacle["width"], obstacle["y"] + obstacle["height"]
                ], fill=debris_color, outline=(139, 69, 19), width=1)
                
                # Aggiungi alcuni "rami"
                for i in range(2):
                    line_start = (obstacle["x"] + np.random.randint(0, obstacle["width"]),
                                 obstacle["y"] + np.random.randint(0, obstacle["height"]))
                    line_end = (line_start[0] + np.random.randint(-10, 10),
                               line_start[1] + np.random.randint(-10, 10))
                    draw.line([line_start, line_end], fill=(101, 67, 33), width=3)
    
    def _draw_grass_patches(self, draw):
        """Disegna patch di erba dense e realistiche"""
        for patch in self.grass_patches:
            # Aggiorna posizione
            patch["x"] += patch["dx"]
            patch["y"] += patch["dy"]
            
            # Rimbalza sui bordi (con margine per il perimetro)
            margin = 60
            if patch["x"] <= margin or patch["x"] >= self.width - margin - patch["size"]:
                patch["dx"] *= -1
            if patch["y"] <= margin or patch["y"] >= self.height - margin - patch["size"]:
                patch["dy"] *= -1
            
            # Colore erba basato sulla densità
            density = patch["density"]
            if density > 0.8:
                grass_color = (34, 139, 34)  # Verde intenso
            elif density > 0.6:
                grass_color = (50, 120, 50)  # Verde medio
            else:
                grass_color = (70, 100, 70)  # Verde chiaro
            
            # Disegna patch di erba principale
            draw.ellipse([
                patch["x"], patch["y"],
                patch["x"] + patch["size"], patch["y"] + patch["size"]
            ], fill=grass_color)
            
            # Aggiungi texture erba realistica
            num_blades = int(patch["size"] * density / 3)
            for i in range(num_blades):
                blade_x = patch["x"] + np.random.randint(0, patch["size"])
                blade_y = patch["y"] + np.random.randint(0, patch["size"])
                blade_height = np.random.randint(3, 8)
                
                # Filo d'erba
                draw.line([
                    (blade_x, blade_y),
                    (blade_x + np.random.randint(-1, 2), blade_y - blade_height)
                ], fill=(0, 100, 0), width=1)
    
    def _draw_debug_info(self, draw):
        """Disegna informazioni di debug sull'immagine"""
        # Timestamp e frame info
        timestamp_text = f"Frame: {self.frame_count:04d} | Time: {time.strftime('%H:%M:%S')}"
        draw.text((10, 10), timestamp_text, fill=(255, 255, 255))
        
        # Contatori elementi
        elements_text = f"Grass: {len(self.grass_patches)} | Obstacles: {len(self.obstacles)} | Perimeter: {len(self.perimeter_lines)}"
        draw.text((10, 30), elements_text, fill=(255, 255, 0))
        
        # Indicatore di movimento
        move_indicator = f"Comprehensive Vision Test Scene"
        draw.text((10, self.height - 50), move_indicator, fill=(255, 255, 255))
        
        # Legenda colori
        legend_text = "[Brown] Perimeter | [Gray] Obstacles | [Green] Grass"
        draw.text((10, self.height - 30), legend_text, fill=(200, 200, 200))
    
    def image_to_base64(self, img):
        """Converte immagine PIL in base64"""
        buffer = io.BytesIO()
        img.save(buffer, format='JPEG', quality=85)
        img_bytes = buffer.getvalue()
        return base64.b64encode(img_bytes).decode('utf-8')
    
    def publish_frame(self):
        """Pubblica un frame su MQTT"""
        try:
            # Crea immagine sintetica
            img = self.create_synthetic_image()
            
            # Converte in base64
            img_b64 = self.image_to_base64(img)
            
            # Crea messaggio MQTT compatibile con grass detector
            message = {
                "type": "camera_frame",
                "timestamp": int(time.time() * 1000),
                "frame_id": self.frame_count,
                "width": self.width,
                "height": self.height,
                "format": "jpeg",
                "encoding": "base64",
                "image": img_b64,  # Campo 'image' richiesto dal grass detector
                "metadata": {
                    "camera_id": "simulator",
                    "fps": 2.0,
                    "exposure": "auto",
                    "focus": "auto"
                }
            }
            
            # Pubblica su topic corretto per grass detector
            topic = "smartmower/vision/camera"
            self.client.publish(topic, json.dumps(message))
            
            print(f"📸 Frame {self.frame_count:04d} pubblicato su {topic} ({len(img_b64)} bytes)")
            
        except Exception as e:
            print(f"❌ Errore pubblicazione frame: {e}")
    
    def run_simulation(self, fps=2.0, duration_minutes=5):
        """Esegue la simulazione per la durata specificata"""
        print(f"🎬 Avvio simulazione camera:")
        print(f"   FPS: {fps}")
        print(f"   Durata: {duration_minutes} minuti")
        print(f"   Topic: smartmower/vision/camera")
        
        # Connetti al broker
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.client.loop_start()
        except Exception as e:
            print(f"❌ Errore connessione MQTT: {e}")
            return
        
        frame_interval = 1.0 / fps
        start_time = time.time()
        end_time = start_time + (duration_minutes * 60)
        
        try:
            while self.running and time.time() < end_time:
                self.publish_frame()
                time.sleep(frame_interval)
                
        except KeyboardInterrupt:
            print("\n⏹️  Simulazione interrotta dall'utente")
        
        # Statistiche finali
        elapsed = time.time() - start_time
        print(f"\n📊 Simulazione completata:")
        print(f"   Durata: {elapsed:.1f} secondi")
        print(f"   Frame pubblicati: {self.frame_count}")
        print(f"   FPS medio: {self.frame_count/elapsed:.2f}")
        
        self.client.loop_stop()
        self.client.disconnect()
    
    def stop(self):
        """Ferma la simulazione"""
        self.running = False

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Simulatore camera per grass detector')
    parser.add_argument('--fps', type=float, default=2.0, help='Frame per secondo (default: 2.0)')
    parser.add_argument('--duration', type=int, default=5, help='Durata in minuti (default: 5)')
    parser.add_argument('--width', type=int, default=640, help='Larghezza immagine (default: 640)')
    parser.add_argument('--height', type=int, default=480, help='Altezza immagine (default: 480)')
    
    args = parser.parse_args()
    
    simulator = CameraSimulator()
    simulator.width = args.width
    simulator.height = args.height
    
    try:
        simulator.run_simulation(fps=args.fps, duration_minutes=args.duration)
    except Exception as e:
        print(f"❌ Errore durante simulazione: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
