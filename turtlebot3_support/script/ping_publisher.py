#!/usr/bin/env python3

import os
import sys
import rclpy

from geometry_msgs.msg import Vector3Stamped
from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

class PingPublisher(Node):
    def __init__(self):
        super().__init__('ping_publisher')
        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Vector3Stamped, 'ping', qos)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Ping publisher test')

def main():
    try:
        rclpy.init()
        node = PingPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
