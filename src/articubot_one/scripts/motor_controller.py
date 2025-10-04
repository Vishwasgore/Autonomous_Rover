#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # L298N GPIO Pins - ADJUST THESE FOR YOUR ROBOT!
        self.IN1 = 17  # Left Motor Forward
        self.IN2 = 18  # Left Motor Backward
        self.IN3 = 22  # Right Motor Forward  
        self.IN4 = 23  # Right Motor Backward
        self.ENA = 12  # Left Motor PWM
        self.ENB = 13  # Right Motor PWM
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.IN1, self.IN2, self.IN3, self.IN4, self.ENA, self.ENB], GPIO.OUT)
        
        # PWM Setup
        self.pwm_left = GPIO.PWM(self.ENA, 1000)  # 1kHz frequency
        self.pwm_right = GPIO.PWM(self.ENB, 1000)
        self.pwm_left.start(0)
        self.pwm_right.start(0)
        
        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.get_logger().info('GPIO Motor Controller Ready - Pins configured')
    
    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Differential drive calculation
        left_vel = linear - angular * 0.1
        right_vel = linear + angular * 0.1
        
        # Convert to PWM (0-100)
        left_pwm = max(-100, min(100, left_vel * 50))
        right_pwm = max(-100, min(100, right_vel * 50))
        
        # Control motors via GPIO
        self.drive_motors(left_pwm, right_pwm)
        
        self.get_logger().info(f'GPIO Control - L: {left_pwm:.1f}%, R: {right_pwm:.1f}%')
    
    def drive_motors(self, left_pwm, right_pwm):
        # Left motor
        if left_pwm >= 0:
            GPIO.output(self.IN1, GPIO.HIGH)
            GPIO.output(self.IN2, GPIO.LOW)
            self.pwm_left.ChangeDutyCycle(abs(left_pwm))
        else:
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.HIGH)
            self.pwm_left.ChangeDutyCycle(abs(left_pwm))
        
        # Right motor
        if right_pwm >= 0:
            GPIO.output(self.IN3, GPIO.HIGH)
            GPIO.output(self.IN4, GPIO.LOW)
            self.pwm_right.ChangeDutyCycle(abs(right_pwm))
        else:
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.HIGH)
            self.pwm_right.ChangeDutyCycle(abs(right_pwm))
    
    def destroy_node(self):
        GPIO.cleanup()
        self.get_logger().info('GPIO cleanup completed')
        super().destroy_node()

def main():
    rclpy.init()
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()