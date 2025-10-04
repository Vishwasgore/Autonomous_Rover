#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # L298N GPIO Pins - ENA/ENB connected to 5V (always enabled)
        self.IN1 = 17  # Left Motor Forward
        self.IN2 = 18  # Left Motor Backward
        self.IN3 = 22  # Right Motor Forward  
        self.IN4 = 23  # Right Motor Backward
        # REMOVED: self.ENA = 12  # Connected to 5V
        # REMOVED: self.ENB = 13  # Connected to 5V
        
        # Setup GPIO - ONLY control pins needed
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.IN1, self.IN2, self.IN3, self.IN4], GPIO.OUT)
        
        # NO PWM setup needed since ENA/ENB are connected to 5V
        
        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.get_logger().info('GPIO Motor Controller Ready - ENA/ENB connected to 5V')
    
    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Differential drive calculation
        left_vel = linear - angular * 0.1
        right_vel = linear + angular * 0.1
        
        # Convert to simple forward/backward/stop (no speed control)
        left_direction = self._velocity_to_direction(left_vel)
        right_direction = self._velocity_to_direction(right_vel)
        
        # Control motors via GPIO
        self.drive_motors(left_direction, right_direction)
        
        self.get_logger().info(f'Motor Control - L: {left_direction}, R: {right_direction}')
    
    def _velocity_to_direction(self, velocity):
        """Convert velocity to motor direction"""
        if velocity > 0.1:  # Forward
            return "FORWARD"
        elif velocity < -0.1:  # Backward
            return "BACKWARD"
        else:  # Stop
            return "STOP"
    
    def drive_motors(self, left_dir, right_dir):
        """Control motors via GPIO - ENA/ENB always enabled"""
        
        # Left motor control
        if left_dir == "FORWARD":
            GPIO.output(self.IN1, GPIO.HIGH)
            GPIO.output(self.IN2, GPIO.LOW)
        elif left_dir == "BACKWARD":
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.HIGH)
        else:  # STOP
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.LOW)
        
        # Right motor control
        if right_dir == "FORWARD":
            GPIO.output(self.IN3, GPIO.HIGH)
            GPIO.output(self.IN4, GPIO.LOW)
        elif right_dir == "BACKWARD":
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.HIGH)
        else:  # STOP
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.LOW)
    
    def destroy_node(self):
        # Stop all motors and cleanup
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
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