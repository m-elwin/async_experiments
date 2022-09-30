""" Different examples for how to handle (both properly and improperly) asynchronous calls """
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from enum import Enum, auto
from rclpy.callback_groups import ReentrantCallbackGroup

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
        self.cbgroup = ReentrantCallbackGroup()
        # Add client and timer to a ReentrantCallback group so timer and the client future can execute concurrently
        # This code would also work if the client and timer were in different MutuallyExclusiveCallback groups
        self._client = self.create_client(Empty, "delay", callback_group = self.cbgroup)
        self._tmr = self.create_timer(5.0, self.timer_callback, callback_group = self.cbgroup)

    async def timer_callback(self):
        """ We have made the timer callback asynchronous, so it can give up execution time to other tasks """
        self.get_logger().info("Timer, calling delay 3s")
        # When we await the future, execution of timer_callback is suspended, allowing the main ROS spin loop to
        # Keep spinning and update the timer.
        # Timer and the client must be in a ReentrantCallback group
        await self._client.call_async(Empty.Request())
        self.get_logger().info("Timer Done!")



class FutureClient(Node):
    """ Check on the future in each timer iteration """
    def __init__(self):
        super().__init__("future_client")
        self._client = self.create_client(Empty, "delay")
        self._tmr = self.create_timer(1.0, self.timer_callback)
        self._future = None

    def timer_callback(self):
        if not self._future: # A call is not pending
            self.get_logger().info("Timer, calling delay 3s")
            self._future = self._client.call_async(Empty.Request())

        if self._future.done(): # a call just completed
            self.get_logger().info("Timer Done!")
            self._future = None
        else:
                self.get_logger().info("Doing other stuff!")

class State(Enum):
    """ States for tracking the completion of the service call """
    DELAY = auto(),
    DONE = auto()

class YieldClient(Node):
    def __init__(self):
        super().__init__("yield_client")
        self._client = self.create_client(Empty, "delay")
        self._tmr = self.create_timer(1.0, self.timer_callback)
        self._delay_generator = None
        self.state = State.DONE

    def _delay(self):
        """ Makes an asynchronous delay call, and repeatedly yields until the future is done """
        # This pattern can be followed for any service client
        # Essentially, this function makes a request, then checks the future
        # If the future is not done(), it yields
        # yielding causes _delay() to return an iterator.
        # calling next() on the iterator will resume _delay() from the point
        # where it last called yield
        future = self._client.call_async(Empty.Request())
        while not future.done():
            yield

    def _double_delay(self):
        """ Make two delay calls in a row """
        # This pattern enables calling multiple services in a row and yielding until they are each complete
        # yield from will cause this function to yield if the called function yields.
        # when calling next() on the iterator, execution continues from the last yield call
        yield from self._delay()
        yield from self._delay()

    def double_delay(self):
        """ On the first call, call the delay service twice.
            On subsequent calls: if service responses have not yet been received, return False
            If service responses have been received: return True and reset.

            Return:
              True if all services have received responses, False otherwise
        """
        # This function follows a pattern that enables a user to continuously call this function and check
        # it's return value. The function will continue advancing it's work little by little on each
        # subsequent call, until the work is finished
        try:
            if not self._delay_generator:
                # we have not yet made a call to delay so call it
                self._delay_generator = self._double_delay()
            else:
                # a call is pending, check if it's done
                next(self._delay_generator)
            return False
        except StopIteration:
            # Delay has completed so reset
            self._delay_generator = None
            return True

    def timer_callback(self):
        if self.state == State.DONE:
            self.get_logger().info("Timer, calling delay twice, 6s")
            self.state = State.DELAY
        elif self.state == State.DELAY:
            if self.double_delay():
                self.get_logger().info("Timer Done!")
                self.state = State.DONE
            else:
                self.get_logger().info("Doing other stuff!")

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

def future_entry(args=None):
    rclpy.init(args=args)
    node = FutureClient()
    rclpy.spin(node)
    rclpy.shutdown()

def yield_entry(args=None):
    rclpy.init(args=args)
    node = YieldClient()
    rclpy.spin(node)
    rclpy.shutdown()
