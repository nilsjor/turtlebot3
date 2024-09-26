#!/usr/bin/env python3

import os
import sys
import rclpy
import subprocess

from geometry_msgs.msg import Vector3Stamped
from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

class PingPublisher(Node):
    def __init__(self):
        super().__init__('ping_publisher')
        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Vector3Stamped, 'ping', qos)
        self.timer = self.create_timer(1/50, self.timer_callback)

    def timer_callback(self):
        result = subprocess.run(
            ['ping', '-6', '-c', '1', '-W', '0.500', 'turtlebot'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        if result.returncode == 0:
            output = result.stdout
            time_ms = float(output.split('time=')[1].split(' ms')[0])
        else:
            time_ms = float('Inf')

        msg = Vector3Stamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.z = time_ms
        self.pub.publish(msg)
        # self.get_logger().info(f'Published ping time: {time_ms} ms')

def main():
    try:
        rclpy.init()
        node = PingPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
