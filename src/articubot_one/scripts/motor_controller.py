#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import RPi.GPIO as GPIO
import math

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # L298N GPIO Pins
        self.IN1 = 17  # Left Motor Forward
        self.IN2 = 18  # Left Motor Backward
        self.IN3 = 22  # Right Motor Forward  
        self.IN4 = 23  # Right Motor Backward
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.IN1, self.IN2, self.IN3, self.IN4], GPIO.OUT)
        
        # Robot parameters - UPDATED with your specifications
        self.wheel_separation = 0.12  # 12cm between wheels
        self.wheel_radius = 0.04      # 4cm wheel radius
        
        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Current velocities for odometry
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        
        # Publishers
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Timer for odometry publication (20Hz)
        self.timer = self.create_timer(0.05, self.publish_odometry)
        
        self.get_logger().info('GPIO Motor Controller Ready with Odometry')
        self.get_logger().info(f'Wheel separation: {self.wheel_separation}m, Wheel radius: {self.wheel_radius}m')
    
    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Store velocities for odometry
        self.current_linear_vel = linear
        self.current_angular_vel = angular
        
        # CORRECT differential drive calculation
        left_vel = linear - angular * self.wheel_separation / 2.0
        right_vel = linear + angular * self.wheel_separation / 2.0
        
        # Convert to motor directions
        left_direction = self._velocity_to_direction(left_vel)
        right_direction = self._velocity_to_direction(right_vel)
        
        # Control motors
        self.drive_motors(left_direction, right_direction)
        
        self.get_logger().debug(f'Cmd: lin={linear:.2f}, ang={angular:.2f} -> L:{left_direction}, R:{right_direction}')
    
    def _velocity_to_direction(self, velocity):
        """Convert velocity to motor direction"""
        if velocity > 0.05:  # Lower threshold for better response
            return "FORWARD"
        elif velocity < -0.05:
            return "BACKWARD"
        else:
            return "STOP"
    
    def drive_motors(self, left_dir, right_dir):
        """Control motors via GPIO"""
        
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
    
    def publish_odometry(self):
        """Publish odometry data and transform"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt > 0:
            # Update position using open-loop estimation
            delta_x = self.current_linear_vel * math.cos(self.theta) * dt
            delta_y = self.current_linear_vel * math.sin(self.theta) * dt
            delta_theta = self.current_angular_vel * dt
            
            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta
            
            # Normalize theta between -pi and pi
            while self.theta > math.pi:
                self.theta -= 2.0 * math.pi
            while self.theta < -math.pi:
                self.theta += 2.0 * math.pi
            
            self.last_time = current_time
        
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        
        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        cp = math.cos(0 * 0.5)
        sp = math.sin(0 * 0.5)
        cr = math.cos(0 * 0.5)
        sr = math.sin(0 * 0.5)
        
        odom_msg.pose.pose.orientation.w = cy * cp * cr + sy * sp * sr
        odom_msg.pose.pose.orientation.x = cy * cp * sr - sy * sp * cr
        odom_msg.pose.pose.orientation.y = sy * cp * sr + cy * sp * cr
        odom_msg.pose.pose.orientation.z = sy * cp * cr - cy * sp * sr
        
        # Set velocity
        odom_msg.twist.twist.linear.x = self.current_linear_vel
        odom_msg.twist.twist.angular.z = self.current_angular_vel
        
        # Set covariance
        odom_msg.pose.covariance[0] = 0.1  # x
        odom_msg.pose.covariance[7] = 0.1  # y
        odom_msg.pose.covariance[35] = 0.2  # yaw
        
        self.odom_publisher.publish(odom_msg)
        
        # Publish transform
        transform_msg = TransformStamped()
        transform_msg.header.stamp = current_time.to_msg()
        transform_msg.header.frame_id = "odom"
        transform_msg.child_frame_id = "base_footprint"
        transform_msg.transform.translation.x = self.x
        transform_msg.transform.translation.y = self.y
        transform_msg.transform.translation.z = 0.0
        transform_msg.transform.rotation = odom_msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(transform_msg)
    
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
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()