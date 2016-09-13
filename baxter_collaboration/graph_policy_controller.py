import rospy
from baxter_collaboration.srv import DoAction
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


class BaseGPController(object):

    T_WAIT = 1.  # Wait time

    # Base observations
    NONE = 'none'
    YES = 'yes'
    NO = 'no'
    ERROR = 'error'

    def __init__(self, policy_runner):
        self.pr = policy_runner
        self.finished = False
        # ROS stuff
        rospy.init_node("hold_and_release_controller")
        rospy.wait_for_service(ACTION_SERVICE_LEFT)
        self.action_left = rospy.ServiceProxy(ACTION_SERVICE_LEFT, DoAction)
        rospy.wait_for_service(ACTION_SERVICE_RIGHT)
        self.action_right = rospy.ServiceProxy(ACTION_SERVICE_RIGHT, DoAction)
        rospy.wait_for_service(SPEECH_SERVICE)
        self.speech = rospy.ServiceProxy(SPEECH_SERVICE, Speech)
        self.answer_sub = CommunicationSuscriber(COM_TOPIC)
        self.error_sub = ErrorSuscriber(ERR_TOPIC, timeout=5)

    def say(self, sentence):
        self.speech(SpeechRequest.SAY, sentence, None)

    def run(self):
        while not self.finished:
            rospy.loginfo("Current state in policy: " + str(self.pr.current))
            obs = self.take_action(self.pr.get_action())
            rospy.loginfo("Observed: " + obs)
            self.pr.step(obs)

    def take_action(self, action):
        raise NotImplemented

    def action_wait(self):
        rospy.loginfo("Waiting")
        rospy.sleep(self.T_WAIT)
        return self.NONE
