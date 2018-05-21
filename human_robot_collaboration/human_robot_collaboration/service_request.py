"""Provides helper class for asynchronous service calls."""


import threading
import rospy
import time


class FakeServiceRequest:

    finished = True

    def wait_result(self):
        rospy.logdebug("FakeServiceRequest wait_result {}")
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
        thread_name = threading.currentThread().getName()

        rospy.logdebug("{} Start Thread {}".format(thread_name, self.args))
        self.start()
        rospy.logdebug("{} End   Thread {}".format(thread_name, self.args))

    def run(self):
        thread_name = threading.currentThread().getName()

        rospy.logdebug("{} Calling proxy {}".format(thread_name, self.args))
        self.result = self.proxy(*self.args)
        rospy.logdebug("{} Called  proxy {}".format(thread_name, self.args))

        self.finished = True

    def wait_result(self):
        """Blocks until service has returned and returns result.

        Equivalent to a regular (synchronous) service call.
        """
        thread_name = threading.currentThread().getName()
        start_time  = time.time()

        rospy.logdebug("{} Args {}".format(thread_name, self.args))
        self.join()
        rospy.logdebug("{} Elapsed time {}".format(thread_name, time.time()-start_time))

        return self.result
