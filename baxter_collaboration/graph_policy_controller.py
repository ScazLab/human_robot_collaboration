import rospy
from baxter_collaboration.srv import DoAction
from baxter_collaboration.timer import Timer
from baxter_collaboration.service_request import ServiceRequest
from baxter_collaboration.suscribers import (CommunicationSuscriber,
                                             ErrorSuscriber)
from svox_tts.srv import Speech, SpeechRequest


COM_TOPIC = '/web_interface'
ERR_TOPIC = '/robot/digital_io/left_lower_button/state'
SPEECH_SERVICE = '/svox_tts/speech'
ACTION_SERVICE_LEFT = '/action_provider/service_left'
ACTION_SERVICE_RIGHT = '/action_provider/service_right'
RELEASE = 'release'
RECOVER = 'recover'


def fake_service_proxy(*args):
    return True


class BaseGPController(object):

    T_WAIT = 1.  # Wait time

    # Base observations
    NONE = 'none'
    YES = 'yes'
    NO = 'no'
    ERROR = 'error'

    HOME = 'home'

    def __init__(self, policy_runner, timer_path=None, right=True):
        self.pr = policy_runner
        self.finished = False
        # ROS stuff
        rospy.init_node("hold_and_release_controller")
        rospy.loginfo('Waiting for left service...')
        rospy.wait_for_service(ACTION_SERVICE_LEFT)
        self.action_left = rospy.ServiceProxy(ACTION_SERVICE_LEFT, DoAction)
        if right:
            rospy.loginfo('Waiting for right service...')
            rospy.wait_for_service(ACTION_SERVICE_RIGHT)
            self.action_right = rospy.ServiceProxy(ACTION_SERVICE_RIGHT, DoAction)
        else:
            self.action_right = fake_service_proxy 
        rospy.loginfo('Waiting for speech service...')
        rospy.wait_for_service(SPEECH_SERVICE)
        self.speech = rospy.ServiceProxy(SPEECH_SERVICE, Speech)
        self.answer_sub = CommunicationSuscriber(COM_TOPIC, self._stop)
        self.error_sub = ErrorSuscriber(ERR_TOPIC, timeout=5)
        self._say_req = None
        self.timer = Timer(path=timer_path)
        rospy.loginfo('Done.')
        self._home()

    def _home(self):
        rospy.loginfo('Going home befor starting.')
        l = ServiceRequest(self.action_left, self.HOME, 0)
        r = ServiceRequest(self.action_right, self.HOME, 0)
        l.wait_result()
        r.wait_result()

    def say(self, sentence, sync=True):
        prev = self._say_req
        if prev is not None and not prev.finished:
            rospy.loginfo('Waiting for end of previous speech utterance.')
            prev.wait_result()
        self._say_req = ServiceRequest(
            self.speech, SpeechRequest.SAY, sentence, None)
        if sync:
            self._say_req.join()
        else:
            return self._say_req

    def _stop(self):
        if not self.finished:
            rospy.loginfo('Stopping policy controller')
            self.timer.log('Stop')
            rospy.loginfo(str(self.timer.data))
            self.timer.save()
            self.finished = True

    def run(self):
        self.timer.start()
        obs = None
        while not self.finished:
            try:
                rospy.loginfo("Current state in policy: " + str(self.pr.current))
                self.timer.log({'node': self.pr.current,
                                'action': self.pr.get_action(),
                                'last-observation': obs,
                                })
                obs = self.take_action(self.pr.get_action())
                rospy.loginfo("Observed: " + obs)
                self.pr.step(obs)
            except Exception as e:
                rospy.logerr(e)
                self.finished = True

    def take_action(self, action):
        raise NotImplemented

    def action_wait(self):
        rospy.loginfo("Waiting")
        rospy.sleep(self.T_WAIT)
        return self.NONE
