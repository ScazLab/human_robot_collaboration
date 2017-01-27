import rospy

from .controller import BaseController


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
