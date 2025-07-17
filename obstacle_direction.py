#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class ScanSubscriber(Node):
    def __init__(self):
        super().__init__('scan_subscriber')
        qos_profile_sensor_data = rclpy.qos.qos_profile_sensor_data
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        self.get_logger().info('Scan subscriber node has been started.')

    def scan_callback(self, msg):
        self.scan_ranges = np.array(msg.ranges)
        min_distance = np.nanmin(self.scan_ranges)
        min_distance_index = np.nanargmin(self.scan_ranges)
        min_distance_angle = np.degrees(min_distance_index * msg.angle_increment + msg.angle_min)
        self.get_logger().info(f'Distance to nearest obstacle: {min_distance:.2f} m at {min_distance_angle:.2f} degrees')

def main(args=None):
    rclpy.init(args=args)
    node = ScanSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
