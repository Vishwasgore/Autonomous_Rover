#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros

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
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
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
        
        # Add some covariance
        odom.pose.covariance = [0.1] * 36  # Simplified covariance
        odom.twist.covariance = [0.1] * 36
        
        self.odom_pub.publish(odom)
        
        # Also publish TF transform
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.orientation
        
        self.tf_broadcaster.sendTransform(t)

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