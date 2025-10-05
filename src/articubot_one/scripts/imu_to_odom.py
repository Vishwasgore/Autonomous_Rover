#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class ImuToOdom(Node):
    def __init__(self):
        super().__init__('imu_to_odom')
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscriber
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/filtered',
            self.imu_callback,
            10
        )
        
        self.get_logger().info("IMU to Odometry converter started")
        
    def imu_callback(self, msg):
        # Only publish if we have valid orientation data
        if (msg.orientation.x == 0.0 and msg.orientation.y == 0.0 and 
            msg.orientation.z == 0.0 and msg.orientation.w == 0.0):
            return
            
        try:
            # Create odometry message
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'
            
            # Copy orientation from IMU
            odom.pose.pose.orientation = msg.orientation
            
            # Set position to zero (IMU can't measure position)
            odom.pose.pose.position.x = 0.0
            odom.pose.pose.position.y = 0.0
            odom.pose.pose.position.z = 0.0
            
            # Copy angular velocity
            odom.twist.twist.angular = msg.angular_velocity
            
            # Set linear velocity to zero (we don't have this from IMU)
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.linear.z = 0.0
            
            # Add covariance (required field)
            odom.pose.covariance = [
                1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.1
            ]
            
            odom.twist.covariance = [
                0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.05, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.05
            ]
            
            self.odom_pub.publish(odom)
            
            # Publish TF transform from odom to base_footprint
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation = msg.orientation
            
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().error(f"Error in IMU callback: {e}")

def main():
    rclpy.init()
    node = ImuToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()