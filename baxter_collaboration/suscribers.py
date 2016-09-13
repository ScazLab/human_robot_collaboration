import rospy
from std_msgs.msg import String

from baxter_core_msgs.msg import DigitalIOState


class WaitForOneSuscriber:
    """Suscriber to wait for an expected message.
    """

    period = .1

    def __init__(self, topic, timeout=10.):
        self._suscribe(topic)
        self.listening = False
        self.timeout = timeout

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

    def wait_for_msg(self, timeout=None):
        if timeout is None:
            timeout = self.timeout
        self._reset_msg()
        self.listening = True
        start_time = rospy.Time.now()
        while (self.last_msg is None and
               rospy.Time.now() < start_time + rospy.Duration(timeout)):
            rospy.sleep(self.period)
        self.listening = False
        return self.last_msg


class CommunicationSuscriber(WaitForOneSuscriber):

    def _suscribe(self, topic):
        self.sub = rospy.Subscriber(topic, String, self.cb)

    def _handle_msg(self, msg):
        self.last_msg = msg.data


class ErrorSuscriber(WaitForOneSuscriber):

    def _suscribe(self, topic):
        self.sub = rospy.Subscriber(topic, DigitalIOState, self.cb)

    def _handle_msg(self, msg):
        if msg.state == DigitalIOState.PRESSED:
            self.last_msg = True
        else:
            pass
