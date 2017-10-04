from threading import Lock

import rospy
from std_msgs.msg import String

from baxter_core_msgs.msg import DigitalIOState
from ros_speech2text.msg import transcript


class WaitForOneSubscriber(object):
    """Subscriber to wait for an expected message.
    """

    period = .1

    def __init__(self, topic, timeout=10.):
        self._lock = Lock()
        self._reset_msg()
        self._listening = False
        self.timeout = timeout
        self._subscribe(topic)

    def _subscribe(self, topic):
        raise NotImplementedError()

    def _reset_msg(self):
        self.last_msg = None

    def _handle_msg(self, msg):
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
        self.stop_listening(found=value)

    def wait_for_msg(self, timeout=None, continuing=False):
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

    def _subscribe(self, topic):
        self.sub = rospy.Subscriber(topic, transcript, self.cb)

    def _handle_msg(self, msg):
        self.found_message(msg.transcript)


class ButtonSubscriber(WaitForOneSubscriber):

    def _subscribe(self, topic):
        self.sub = rospy.Subscriber(topic, DigitalIOState, self.cb)

    def _handle_msg(self, msg):
        if msg.state == DigitalIOState.PRESSED:
            self.found_message(True)
        else:
            pass
