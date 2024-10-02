""" Different examples for how to handle (both properly and improperly) asynchronous calls """
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from enum import Enum, auto
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class DeadlockClient(Node):
    """ This Node will purposely deadlock (which is bad!) """
    def __init__(self):
        super().__init__("async_client")
        self._client = self.create_client(Empty, "delay")
        self._tmr = self.create_timer(5.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Timer, calling delay 3s")
        future = self._client.call_async(Empty.Request())
        self.get_logger().info("Call finished, waiting")

        # Because this call is in the timer_callback, we are not spinning
        # Therefore the future can never be set to the done state
        while not future.done():
            self.get_logger().info("Not Done", once=True)
            # rclpy.spin_once(self) # try it! won't help because we are already in a MutallyExclusive callback
        # We will never get here due to the deadlock
        self.get_logger().info("Timer Done!")

class AwaitClient(Node):
    """ This node uses await and callback groups to properly wait for the service to end """
    def __init__(self):
        super().__init__("async_client")
        self.cbgroup = MutuallyExclusiveCallbackGroup()
        # Service clients go in the MutuallyExclusive callback group
        self._client = self.create_client(Empty, "delay", callback_group = self.cbgroup)
        # Other callbacks remain in the default callback group
        self._tmr = self.create_timer(5.0, self.timer_callback)

    async def timer_callback(self):
        """ We have made the timer callback asynchronous, so it can give up execution time to other tasks """
        self.get_logger().info("Timer, calling delay 3s")
        # When we await the future, execution of timer_callback is suspended, allowing the main ROS spin loop to
        # Keep spinning and update the timer.
        # Timer and the client must be in a ReentrantCallback group
        await self._client.call_async(Empty.Request())
        self.get_logger().info("Timer Done!")


class State(Enum):
    """States for tracking the completion of the service call."""

    DELAY = auto(),
    """The node should call the delay service."""

    WAITING = auto(),
    """The node is waiting for the delay service to return."""

    DONE = auto()
    """The delay service has returned."""

class FutureClient(Node):
    """Check on the future in each timer iteration."""
    def __init__(self):
        super().__init__("future_client")
        self._client = self.create_client(Empty, 'delay')
        self._tmr = self.create_timer(1.0, self.timer_callback)
        self._future = None
        self._state = State.DELAY

    def timer_callback(self):
        """Manage the state machine for the node."""
        if self._state == State.DELAY:
            self.get_logger().info('Initiating a 3s delay.')
            self._future = self._client.call_async(Empty.Request())
            self._state = State.WAITING
        elif self._state == State.WAITING:
            self.get_logger().info('Waiting. A real node could be doing useful work right now.')
            if self._future.done():
                self._state = State.DONE
        elif self._state == State.DONE:
            self.get_logger().info('We have received the delay response.')
            self._state = State.DELAY
        else:
            # Always raise an error if the code somehow gets into an invalid state.
            raise RuntimeError("Invalid State.")


def deadlock_entry(args=None):
    rclpy.init(args=args)
    node = DeadlockClient()
    node.get_logger().info("Deadlock Experiment!")
    rclpy.spin(node)
    rclpy.shutdown()

def await_entry(args=None):
    rclpy.init(args=args)
    node = AwaitClient()
    node.get_logger().info("Await Experiment!")
    rclpy.spin(node)
    rclpy.shutdown()

def future_entry(args=None):
    rclpy.init(args=args)
    node = FutureClient()
    node.get_logger().info("Future Experiment!")
    rclpy.spin(node)
    rclpy.shutdown()
