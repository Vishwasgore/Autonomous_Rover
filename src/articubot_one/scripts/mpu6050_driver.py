#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import smbus2
import math
import time

class MPU6050Driver(Node):
    def __init__(self):
        super().__init__('mpu6050_driver')
        
        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)
        
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # ========== YOUR CALIBRATION DATA ==========
        # Gyroscope offsets (in raw sensor units)
        self.gyro_offset_x = -1.69418
        self.gyro_offset_y = 1.98037
        self.gyro_offset_z = -0.547328
        
        # Accelerometer offsets (in raw sensor units)
        self.accel_offset_x = -1.6805
        self.accel_offset_y = 0.413677
        self.accel_offset_z = 10.4005
        # ===========================================
        
        self.get_logger().info("Using pre-calibrated IMU offsets")
        self.get_logger().info(f"Gyro offsets: X={self.gyro_offset_x}, Y={self.gyro_offset_y}, Z={self.gyro_offset_z}")
        self.get_logger().info(f"Accel offsets: X={self.accel_offset_x}, Y={self.accel_offset_y}, Z={self.accel_offset_z}")
        
        # IMU publisher
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        
        # Initialize I2C
        try:
            self.bus = smbus2.SMBus(self.i2c_bus)
            self.get_logger().info(f"Connected to MPU6050 on I2C bus {self.i2c_bus}, address {hex(self.i2c_address)}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C bus: {e}")
            return
        
        # Initialize MPU6050
        self.init_mpu6050()
        
        # Timer for publishing
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_imu_data)
        
        self.get_logger().info('MPU6050 driver started successfully with pre-calibration')
    
    def init_mpu6050(self):
        """Initialize MPU6050 registers"""
        try:
            # Wake up MPU6050 (PWR_MGMT_1 register)
            self.bus.write_byte_data(self.i2c_address, 0x6B, 0x00)
            
            # Configure gyroscope range (±250°/s)
            self.bus.write_byte_data(self.i2c_address, 0x1B, 0x00)
            
            # Configure accelerometer range (±2g)
            self.bus.write_byte_data(self.i2c_address, 0x1C, 0x00)
            
            # Configure low pass filter
            self.bus.write_byte_data(self.i2c_address, 0x1A, 0x06)
            
            time.sleep(0.1)
            self.get_logger().info("MPU6050 initialized successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MPU6050: {e}")
    
    def read_word(self, reg_high, reg_low):
        """Read two bytes and combine them"""
        high = self.bus.read_byte_data(self.i2c_address, reg_high)
        low = self.bus.read_byte_data(self.i2c_address, reg_low)
        value = (high << 8) + low
        
        # Convert to signed value
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value
    
    def read_accelerometer_raw(self):
        """Read raw accelerometer data"""
        x = self.read_word(0x3B, 0x3C)
        y = self.read_word(0x3D, 0x3E)
        z = self.read_word(0x3F, 0x40)
        return [x, y, z]
    
    def read_gyroscope_raw(self):
        """Read raw gyroscope data"""
        x = self.read_word(0x43, 0x44)
        y = self.read_word(0x45, 0x46)
        z = self.read_word(0x47, 0x48)
        return [x, y, z]
    
    def publish_imu_data(self):
        try:
            # Read raw data
            accel_raw = self.read_accelerometer_raw()
            gyro_raw = self.read_gyroscope_raw()
            
            # Apply YOUR calibration offsets
            accel_x = (accel_raw[0] - self.accel_offset_x) / 16384.0  # ±2g range
            accel_y = (accel_raw[1] - self.accel_offset_y) / 16384.0
            accel_z = (accel_raw[2] - self.accel_offset_z) / 16384.0
            
            gyro_x = (gyro_raw[0] - self.gyro_offset_x) / 131.0  # ±250°/s range
            gyro_y = (gyro_raw[1] - self.gyro_offset_y) / 131.0
            gyro_z = (gyro_raw[2] - self.gyro_offset_z) / 131.0
            
            # Convert to radians
            gyro_x_rad = math.radians(gyro_x)
            gyro_y_rad = math.radians(gyro_y)
            gyro_z_rad = math.radians(gyro_z)
            
            # Create IMU message
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id
            
            # Linear acceleration (m/s²)
            imu_msg.linear_acceleration.x = accel_x * 9.81  # Convert g to m/s²
            imu_msg.linear_acceleration.y = accel_y * 9.81
            imu_msg.linear_acceleration.z = accel_z * 9.81
            
            # Angular velocity (rad/s)
            imu_msg.angular_velocity.x = gyro_x_rad
            imu_msg.angular_velocity.y = gyro_y_rad
            imu_msg.angular_velocity.z = gyro_z_rad
            
            # FIXED: Proper covariance matrices (must be lists with exactly 9 floats)
            # Linear acceleration covariance (m/s²)²
            imu_msg.linear_acceleration_covariance = [
                0.04, 0.0, 0.0,
                0.0, 0.04, 0.0, 
                0.0, 0.0, 0.04
            ]
            
            # Angular velocity covariance (rad/s)²
            imu_msg.angular_velocity_covariance = [
                0.02, 0.0, 0.0,
                0.0, 0.02, 0.0,
                0.0, 0.0, 0.02
            ]
            
            # Orientation covariance (rad²) - unknown for MPU6050
            imu_msg.orientation_covariance = [
                -1.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0
            ]
            
            self.imu_pub.publish(imu_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error reading IMU data: {str(e)}')

def main():
    rclpy.init()
    node = MPU6050Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()