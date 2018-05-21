from threading import Lock

import rospy
from std_msgs.msg import String

from baxter_core_msgs.msg import DigitalIOState
from ros_speech2text.msg import transcript


"""Helper classes to handle (a)synchronous access to robot messages.

All subscribers include a `listening` state and only consider messages
when is listening.

- CommunicationSubscriber: expect a message with a `data` attribute, also
    triggers a `stop` callback on message "STOP"
- ListenSubscriber: handle messages with a transcript attribute
- ButtonSubscriber: handle button messages from the robot
"""


class WaitForOneSubscriber(object):
    """Subscriber to wait until a relevant message is received.

    The subscriber ignores all messages when not listening (initial).
    The `_subscribe` method and `_handle_msg` need to be implemented
    in the child class.
    """

    period = .1

    def __init__(self, topic, timeout=10.):
        self._lock = Lock()
        self._reset_msg()
        self._listening = False
        self.timeout = timeout
        self._subscribe(topic)

    def _subscribe(self, topic):
        """Subscribes to the topic and set self.sub as the subscriber,
        and self.cb as its callback.
        """
        raise NotImplementedError()

    def _reset_msg(self):
        self.last_msg = None

    def _handle_msg(self, msg):
        """Handles a new message (while listening) and decides if relevant.

        If appropriate, the method calls `self.found_message` with the relevant
        data.
        """
        raise NotImplementedError()

    def cb(self, msg):
        if self.listening:
            self._handle_msg(msg)
        else:
            pass

    @property
    def listening(self):
        with self._lock:
            return self._listening

    def start_listening(self):
        with self._lock:
            self._reset_msg()
            self._listening = True

    def stop_listening(self, found=None):
        with self._lock:
            self._listening = False
            if found is not None:
                self.last_msg = found

    def found_message(self, value):
        """Stores found value as last message and stop listening."""
        self.stop_listening(found=value)

    def wait_for_msg(self, timeout=None, continuing=False):
        """Wait until a relevant message is found or timeout (if appropriate).

        :param continuing: Boolean
            Whether the subscriber is already listening or needs to be started.
        """
        if timeout is None:
            timeout = self.timeout
        if not continuing:
            self.start_listening()
        start_time = rospy.Time.now()
        while (self.listening and
               rospy.Time.now() < start_time + rospy.Duration(timeout)):
            rospy.sleep(self.period)
        self.stop_listening()
        with self._lock:
            return self.last_msg


class CommunicationSubscriber(WaitForOneSubscriber):
    """Subscriber for messages that have a `data` attribute.
    When it is equal to the `STOP` class variable (default: `"stop"`),
    the callback given as a parameter `stop_cb` is called.
    """

    STOP = 'stop'

    def __init__(self, topic, stop_cb, timeout=10):
        super(CommunicationSubscriber, self).__init__(topic, timeout=timeout)
        self.stop_cb = stop_cb

    def _subscribe(self, topic):
        self.sub = rospy.Subscriber(topic, String, self.cb)

    def _handle_msg(self, msg):
        self.found_message(msg.data)

    def cb(self, msg):
        if msg.data == self.STOP:
            self.stop_cb()
        if self.listening:
            self._handle_msg(msg)
        else:
            pass


class ListenSubscriber(WaitForOneSubscriber):
    """Subscriber for messages with a `transcript` attribute."""

    def _subscribe(self, topic):
        self.sub = rospy.Subscriber(topic, transcript, self.cb)

    def _handle_msg(self, msg):
        self.found_message(msg.transcript)


class ButtonSubscriber(WaitForOneSubscriber):
    """Subscriber to wait for a button pressed on Baxter."""

    def _subscribe(self, topic):
        self.sub = rospy.Subscriber(topic, DigitalIOState, self.cb)

    def _handle_msg(self, msg):
        if msg.state == DigitalIOState.PRESSED:
            self.found_message(True)
        else:
            pass
