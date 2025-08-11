#!/usr/bin/env python3
"""GPIO Tone Generator - Simple tone generation using RPi.GPIO"""

import time
import sys

try:
    import RPi.GPIO as GPIO
    ON_RPI = True
except (ImportError, RuntimeError):
    ON_RPI = False
    print("Running in simulation mode (not on Raspberry Pi)")

class GPIOTone:
    def __init__(self, pin=26):
        self.pin = pin
        self.running = False
        self.initialized = False
        
        if not ON_RPI:
            print(f"[Simulation] Buzzer on pin {pin}")
            return
            
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pin, GPIO.OUT)
            self.initialized = True
        except Exception as e:
            print(f"Error initializing GPIO: {e}")
    
    def play_tone(self, freq, duration, volume=50):
        """Play tone at frequency (Hz) for duration (seconds)"""
        if not 20 <= freq <= 20000:
            raise ValueError("Frequency must be 20-20000Hz")
        
        print(f"Playing {freq}Hz for {duration}s (volume: {volume}%)")
        
        if not self.initialized:
            time.sleep(duration)
            return
            
        period = 1.0 / freq
        half_period = period / 2.0
        cycles = int(freq * duration)
        
        self.running = True
        try:
            for _ in range(cycles):
                if not self.running: break
                GPIO.output(self.pin, GPIO.HIGH)
                time.sleep(half_period)
                if not self.running: break
                GPIO.output(self.pin, GPIO.LOW)
                time.sleep(half_period)
        except KeyboardInterrupt:
            self.stop()
    
    def stop(self):
        self.running = False
        if hasattr(self, 'pin'):
            try:
                GPIO.output(self.pin, GPIO.LOW)
            except:
                pass
    
    def cleanup(self):
        self.stop()
        if self.initialized:
            try:
                GPIO.cleanup()
            except:
                pass

def test():
    buzzer = None
    try:
        buzzer = GPIOTone(pin=26)
        notes = [(440,20),(523,20),(659,20),(880,20)]
        for freq, duration in notes:
            buzzer.play_tone(freq, duration)
            time.sleep(0.1)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if buzzer:
            buzzer.cleanup()

if __name__ == "__main__":
    test()
