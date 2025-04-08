import rclpy
from geometry_msgs.msg import TwistStamped, Twist
from rclpy.node import Node

class CmdVelStampedToCmdVel(Node):
    def __init__(self):
        super().__init__('cmd_vel_stamped_to_cmd_vel')
        self.subscription = self.create_subscription(
            TwistStamped,
            'cmd_vel_stamped',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def listener_callback(self, msg):
        # Convert the TwistStamped message to a Twist message
        twist = Twist()
        twist.linear = msg.twist.linear
        twist.angular = msg.twist.angular
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelStampedToCmdVel()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
