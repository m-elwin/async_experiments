import rclpy
from rclpy.node import Node
from time import sleep

from std_srvs.srv import Empty

class DelayServer(Node):
    """
    Services:
      /delay3 (std_srvs/Empty): delay for 3 seconds before returning
    """

    def __init__(self):
        super().__init__("delay_server")
        self._delay_srv = self.create_service(Empty, "delay", self.delay_callback)

    async def delay_callback(self,_, resp):
        self.get_logger().info("Begin Delay")
        sleep(3)
        self.get_logger().info("End Delay")
        return resp


def delay_entry(args=None):
    rclpy.init(args=args)
    node = DelayServer()
    rclpy.spin(node)
    rclpy.shutdown()
