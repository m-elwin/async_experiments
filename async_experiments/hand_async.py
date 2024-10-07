"""
Demonstration of async using regular functions and classes.

Here is how to use

# Make a fifo to receive events
mkfifo /tmp/fifo

# In one terminal window, run this script
python3 handmade_async.py

# In a separate window we will send events.
# Events are sent by echoing characters to /tmp/fifo
# Events understood by this script are
echo "Q" > /tmp/fifo # Quit the script
echo "A" > /tmp/fifo # Print a response to A
echo "C" > /tmp/fifo # initiate a callback that makes an "async" call and waits for a return
echo "B" > /tmp/fifo # provides the script with the "response" to the async call

# Here's an example sequence of events to send
echo "A" > /tmp/fifo # Just to see that we are responding to events
echo "C" > /tmp/fifo # Initiate the "async" callback
echo "A" > /tmp/fifo # We can still respond to other events while waiting for "C" to return
echo "C" > /tmp/fifo # The "callback group" (pending boolean) prevents C from being called again until it is done
echo "B" > /tmp/fifo # The "response" is set. The callback gets the value from the future and prints it

"""

import time



class Future:
    """Storage location for a result that will be available in the future."""
    def __init__(self):
        self.result = None

    def done(self):
        """Return true when the result is ready else return false."""
        return self.result is not None

    def set_result(self, res):
        """Set the result for the future."""
        self.result = res


class AsyncCall:
    """
    Make an asynchronous call that returns a future.
    The future will be registered with the event loop.
    """
    def __init__(self):
        self.fut = None

    def __call__(self):
        self.fut = Future()
        print("Async Call Made.")
        return self.fut

class Callback:
    def __init__(self):
        self.future = None
        self.async_call = AsyncCall()

    def __call__(self):
        if not self.future:
            print("Callback Start")
            self.future = self.async_call()
        else:
            if not self.future.done():
                print("Callback waiting")
            else:
                print(f"Callback Result {self.future.result}")
                self.future = None

class EventLoop:
    """
    Runs a basic event loop.
    Events come in the form of characters read from an external pipe.
    The user can echo characters into the pipe to send an event to the system externally
    """

    def __init__(self, file):
        """ file - the name of the pipe to read from."""
        self.file = file
        self.callback = Callback() # The callback in response to event 'C'
        self.pending = False       # Is the callback pending?

    def spin_once(self):
        """One iteration of the event loop. Return false to indicate an exit event"""
        val = self.file.read(1)
        if val == 'A':
            # Report on an event
            print("val A received")
        elif val == 'C':
            if not self.pending:
                # The callback is triggered. it performs an async call
                self.callback()
                self.pending = True
            else:
                # The callback is still pending. So if it were in a "MutuallyExclusiveCallbackGroup" it would not be called again
                print("Callback still pending, ignoring event")
        elif val == 'B':
            if self.pending:
                # it only makes sense to receive the event if the callback is pending
                # The value of the async call has returned
                self.callback.async_call.fut.set_result('Found B')
                self.callback()
                self.pending = False
        return val != 'Q'

    def spin(self):
        """Enter the event loop. Exit when receiving a Q event."""
        print("Beginning to spin")
        i = 0
        while self.spin_once():
            time.sleep(0.5) # Slow down the loop
            print(f"Spin {i}!")
            i += 1

if __name__ == '__main__':
    # Make an event loop that reads from /tmp/fifo
    ev = EventLoop(open('/tmp/fifo'))
    ev.spin()
