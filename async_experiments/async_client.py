import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

from rclpy.callback_groups import ReentrantCallbackGroup

class DeadlockClient(Node):
    """ This will purposely deadlock """
    def __init__(self):
        super().__init__("async_client")
        self._client = self.create_client(Empty, "delay")
        self._tmr = self.create_timer(5.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Timer, calling delay 3s")
        future = self._client.call_async(Empty.Request())
        self.get_logger().info("Call finished, waiting")
        while not future.done():
            self.get_logger().info("Not Done", once=True)
        # We will never get here because while we are looping in the timer_callback,

        self.get_logger().info("Timer Done!")

class AwaitClient(Node):
    def __init__(self):
        super().__init__("async_client")
        self.cbgroup = ReentrantCallbackGroup()
        self._client = self.create_client(Empty, "delay", callback_group = self.cbgroup)
        self._tmr = self.create_timer(5.0, self.timer_callback, callback_group = self.cbgroup)

    async def timer_callback(self):
        self.get_logger().info("Timer, calling delay 3s")
        await self._client.call_async(Empty.Request())
        self.get_logger().info("Timer Done!")

def deadlock_entry(args=None):
    rclpy.init(args=args)
    node = DeadlockClient()
    rclpy.spin(node)
    rclpy.shutdown()

def await_entry(args=None):
    rclpy.init(args=args)
    node = AwaitClient()
    rclpy.spin(node)
    rclpy.shutdown()
