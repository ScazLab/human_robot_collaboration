import rospy
from baxter_collaboration.srv import DoAction, DoActionRequest
from baxter_collaboration.timer import Timer
from baxter_collaboration.service_request import ServiceRequest
from baxter_collaboration.suscribers import (CommunicationSuscriber,
                                             ErrorSuscriber)
from svox_tts.srv import Speech, SpeechRequest


def fake_service_proxy(*args):
    return True


class BaseController(object):
    """Abstract interaction with robot's ROS nodes.

    Uses the following ressources:
    - service clients for left and right arm action providers,
    - service client for text to speech,
    - suscriber to communication topic,
    - suscriber to error topic.
    """

    NODE_NAME = "experiment_controller"

    COM_TOPIC = '/web_interface/pub'
    ERR_TOPIC = '/robot/digital_io/left_lower_button/state'
    SPEECH_SERVICE = '/svox_tts/speech'
    ACTION_SERVICE_LEFT = '/action_provider/service_left'
    ACTION_SERVICE_RIGHT = '/action_provider/service_right'

    def __init__(self, timer_path=None, left=True, right=True, speech=True):
        self.finished = False
        # ROS stuff
        rospy.init_node(self.NODE_NAME, disable_signals=True)
        if left:  # Left arm action service client
            rospy.loginfo('Waiting for left service...')
            rospy.wait_for_service(self.ACTION_SERVICE_LEFT)
            self.action_left = rospy.ServiceProxy(self.ACTION_SERVICE_LEFT, DoAction)
        else:
            self.action_left = fake_service_proxy
        if right:  # Right arm action service client
            rospy.loginfo('Waiting for right service...')
            rospy.wait_for_service(self.ACTION_SERVICE_RIGHT)
            self.action_right = rospy.ServiceProxy(self.ACTION_SERVICE_RIGHT, DoAction)
        else:
            self.action_right = fake_service_proxy
        if speech:  # Text to speech client
            rospy.loginfo('Waiting for speech service...')
            rospy.wait_for_service(self.SPEECH_SERVICE)
            self.speech = rospy.ServiceProxy(self.SPEECH_SERVICE, Speech)
        self._say_req = None
        # Suscriber to human answers
        self.answer_sub = CommunicationSuscriber(self.COM_TOPIC, self._stop)
        # Suscriber to errors
        self.error_sub = ErrorSuscriber(self.ERR_TOPIC, timeout=5)
        # Timer to log events
        self.timer = Timer(path=timer_path)
        rospy.loginfo('Done.')
        self._home()  # Home position

    def _home(self):
        rospy.loginfo('Going home before starting.')
        l = ServiceRequest(self.action_left, DoActionRequest.ACTION_HOME, [])
        r = ServiceRequest(self.action_right, DoActionRequest.ACTION_HOME, [])
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
            self._home()

    def _abort_waiting_suscribers(self):
        self.answer_sub.listening = False
        self.error_sub.listening = False

    def run(self):
        try:
            self._run()
        except (Exception, KeyboardInterrupt) as e:
            rospy.logerr(e)
            rospy.logerr('Exiting.')
            self.finished = True
            self._abort_waiting_suscribers()
            raise

    def take_action(self, action):
        raise NotImplementedError


class BaseGPController(BaseController):

    NODE_NAME = "graph_policy_controller"

    T_WAIT = 1.  # Wait time

    # Base observations
    NONE = 'none'
    YES = 'yes'
    NO = 'no'
    ERROR = 'error'

    def __init__(self, policy_runner, **kwargs):
        self.pr = policy_runner
        super(BaseGPController, self).__init__(**kwargs)

    def action_wait(self):
        rospy.loginfo("Waiting")
        rospy.sleep(self.T_WAIT)
        return self.NONE

    def _run(self):
        self.timer.start()
        obs = None
        while not self.finished:
            rospy.loginfo("Current state in policy: " + str(self.pr.current))
            self.timer.log({'node': self.pr.current,
                            'action': self.pr.get_action(),
                            'last-observation': obs,
                            })
            obs = self.take_action(self.pr.get_action())
            rospy.loginfo("Observed: " + obs)
            self.pr.step(obs)
