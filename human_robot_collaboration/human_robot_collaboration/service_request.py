"""Provides helper class for asynchronous service calls."""


import threading
import rospy
import time


class FakeServiceRequest:

    finished = True

    def wait_result(self):
        rospy.loginfo("FakeServiceRequest wait_result {}")
        pass


finished_request = FakeServiceRequest()


class ServiceRequest(threading.Thread):
    """Represents a request to a ROS service.

    Call service on creation, in a thread. Stops when service call returns.

    Attribute `finished` indicates status as Boolean.
    Attribute `result` stores return value from service.

    :param service_proxy: service proxy
    :param *args: arguments for service call
    """

    def __init__(self, service_proxy, *args):
        super(ServiceRequest, self).__init__()
        self.proxy = service_proxy
        self.args = args
        self.finished = False
        self.result = None
        rospy.loginfo("{} 1 Constructor {}".format(threading.currentThread().getName(),
                                                   self.args))
        self.start()
        rospy.loginfo("{} 2 Constructor {}".format(threading.currentThread().getName(),
                                                   self.args))

    def run(self, debug=False):
        rospy.loginfo("{} ServiceRequest calling proxy {}".format(threading.currentThread().getName(),
                                                                  self.args))

        self.result = self.proxy(*self.args)
        rospy.loginfo("{} ServiceRequest called  proxy {}".format(threading.currentThread().getName(),
                                                                  self.args))
        self.finished = True

    def wait_result(self):
        """Blocks until service has returned and returns result.

        Equivalent to a regular (synchronous) service call.
        """
        start = time.time()
        rospy.loginfo("{} ServiceRequest wait_result   {}".format(threading.currentThread().getName(),
                                                                  self.args))
        self.join()
        rospy.loginfo("Elapsed time {}".format(time.time()-start))
        return self.result
