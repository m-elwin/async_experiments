"""Creates a service server that can be used to test asynchronous calls."""

from time import sleep

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class DelayServer(Node):
    """
    Server that delays before responding.

    Services
    --------
      delay : std_srvs/srv/Empty - delay for 3 seconds before returning
    """

    def __init__(self):
        super().__init__('delay_server')
        self._delay_srv = self.create_service(Empty, 'delay', self.delay_callback)

    def delay_callback(self, _, resp):
        """Sleep and then return in response to any request."""
        self.get_logger().info('Begin Delay.')
        sleep(3)
        self.get_logger().info('End Delay.')
        return resp


def delay_entry(args=None):
    """Entry point for the delay node."""
    rclpy.init(args=args)
    node = DelayServer()
    rclpy.spin(node)
    rclpy.shutdown()
