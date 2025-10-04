#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

print("Testing GPIO pins for L298N motor driver...")

# L298N GPIO Pins
pins = [17, 18, 22, 23, 12, 13]

GPIO.setmode(GPIO.BCM)
GPIO.setup(pins, GPIO.OUT)

try:
    print("Testing each GPIO pin for 2 seconds...")
    for pin in pins:
        print(f"Testing GPIO {pin} - Motor should move if wired correctly")
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(2)
        GPIO.output(pin, GPIO.LOW)
        time.sleep(0.5)
        
    print("All GPIO tests completed!")
    
finally:
    GPIO.cleanup()
    print("GPIO cleanup done")