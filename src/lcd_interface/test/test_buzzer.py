#!/usr/bin/env python3
"""
Buzzer Test Script for Smart Mower LCD Interface

This script demonstrates how to use the buzzer functionality from the LCD interface.
It plays a series of beeps with different frequencies and volumes.

Usage:
    python3 test_buzzer.py
"""

import os
import sys
import time
import RPi.GPIO as GPIO

# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from lcd_interface import LCDInterface

def test_buzzer():
    """Test function to demonstrate buzzer functionality."""
    lcd = None
    try:
        print("Initializing LCD Interface...")
        lcd = LCDInterface()
        
        # Verify if running on Raspberry Pi
        try:
            import RPi.GPIO as GPIO
            ON_RASPBERRY_PI = True
        except (ImportError, RuntimeError):
            ON_RASPBERRY_PI = False
            print("Warning: Not running on Raspberry Pi. Buzzer will not make sound.")
        
        print("Testing buzzer...")
        
        # Play a simple beep
        print("Playing beep (1000Hz, 50% volume)")
        lcd._beep(1000, 0.5, 50)  # 1000Hz for 0.5s at 50% volume
        
        # Play a rising tone
        if ON_RASPBERRY_PI:
            print("Playing rising tone...")
            for freq in range(200, 1000, 50):
                lcd._beep(freq, 0.1, 30)  # Short beeps for rising tone
                time.sleep(0.05)
        else:
            print("Skipping rising tone (not on Raspberry Pi)")
        
        # Play a series of beeps
        print("Playing beep sequence...")
        for i in range(3):
            lcd._beep(800, 0.2, 70)
            time.sleep(0.1)
        
        # Play a happy tune (simple melody)
        print("Playing simple melody...")
        melody = [
            (262, 0.2),  # C4
            (294, 0.2),  # D4
            (330, 0.2),  # E4
            (392, 0.4),  # G4
            (330, 0.4),  # E4
            (392, 0.8),  # G4
        ]
        
        for freq, duration in melody:
            lcd._beep(freq, duration, 50)
            time.sleep(0.05)  # Small pause between notes
        
        print("Buzzer test completed successfully!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error during buzzer test: {e}")
    finally:
        # Clean up GPIO if we initialized it
        if lcd is not None:
            try:
                # Stop any active PWM
                if hasattr(lcd, 'buzzer'):
                    lcd.buzzer.ChangeDutyCycle(0)
                # Clean up GPIO if we're on Raspberry Pi
                if 'GPIO' in globals():
                    GPIO.cleanup()
            except Exception as e:
                print(f"Error during cleanup: {e}")

if __name__ == "__main__":
    test_buzzer()
