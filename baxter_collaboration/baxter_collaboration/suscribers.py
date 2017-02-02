import rospy
from std_msgs.msg import String

from baxter_core_msgs.msg import DigitalIOState


class WaitForOneSuscriber(object):
    """Suscriber to wait for an expected message.
    """

    period = .1

    def __init__(self, topic, timeout=10.):
        self._suscribe(topic)
        self.listening = False
        self.timeout = timeout
        self._reset_msg()

    def _suscribe(self, topic):
        raise NotImplemented()

    def _reset_msg(self):
        self.last_msg = None

    def _handle_msg(self, msg):
        raise NotImplemented()

    def cb(self, msg):
        if self.listening:
            self._handle_msg(msg)
        else:
            pass

    def start_listening(self):
        self._reset_msg()
        self.listening = True

    def found_message(self, value):
        self.last_msg = value
        self.listening = False

    def wait_for_msg(self, timeout=None, continuing=False):
        if timeout is None:
            timeout = self.timeout
        if not continuing:
            self.start_listening()
        start_time = rospy.Time.now()
        while (self.listening and
               rospy.Time.now() < start_time + rospy.Duration(timeout)):
            rospy.sleep(self.period)
        self.listening = False
        return self.last_msg


class CommunicationSuscriber(WaitForOneSuscriber):

    STOP = 'stop'

    def __init__(self, topic, stop_cb, timeout=10):
        super(CommunicationSuscriber, self).__init__(topic, timeout=timeout)
        self.stop_cb = stop_cb

    def _suscribe(self, topic):
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


class ButtonSuscriber(WaitForOneSuscriber):

    def _suscribe(self, topic):
        self.sub = rospy.Subscriber(topic, DigitalIOState, self.cb)

    def _handle_msg(self, msg):
        if msg.state == DigitalIOState.PRESSED:
            self.found_message(True)
        else:
            pass
