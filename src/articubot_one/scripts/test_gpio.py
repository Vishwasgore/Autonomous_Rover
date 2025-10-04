#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

print("Testing GPIO pins for L298N motor driver (ENA/ENB to 5V)...")

# Only control pins needed (ENA/ENB connected to 5V)
pins = [17, 18, 22, 23]  # IN1, IN2, IN3, IN4

GPIO.setmode(GPIO.BCM)
GPIO.setup(pins, GPIO.OUT)

try:
    print("Testing motor directions (ENA/ENB always enabled)...")
    
    # Test forward
    print("Both motors FORWARD")
    GPIO.output(17, GPIO.HIGH)  # IN1 HIGH
    GPIO.output(18, GPIO.LOW)   # IN2 LOW
    GPIO.output(22, GPIO.HIGH)  # IN3 HIGH
    GPIO.output(23, GPIO.LOW)   # IN4 LOW
    time.sleep(2)
    
    # Test backward
    print("Both motors BACKWARD")
    GPIO.output(17, GPIO.LOW)   # IN1 LOW
    GPIO.output(18, GPIO.HIGH)  # IN2 HIGH
    GPIO.output(22, GPIO.LOW)   # IN3 LOW
    GPIO.output(23, GPIO.HIGH)  # IN4 HIGH
    time.sleep(2)
    
    # Test stop
    print("Both motors STOP")
    GPIO.output(17, GPIO.LOW)   # IN1 LOW
    GPIO.output(18, GPIO.LOW)   # IN2 LOW
    GPIO.output(22, GPIO.LOW)   # IN3 LOW
    GPIO.output(23, GPIO.LOW)   # IN4 LOW
    time.sleep(1)
    
    print("GPIO test completed!")
    
finally:
    GPIO.cleanup()
    print("GPIO cleanup done")