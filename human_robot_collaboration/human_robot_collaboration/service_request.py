"""Provides helper class for asynchronous service calls."""


import threading


class FinishedServiceRequest:

    finished = True

    def wait_result(self):
        pass


finished_request = FinishedServiceRequest()


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
        self.start()

    def run(self):
        self.result = self.proxy(*self.args)
        self.finished = True

    def wait_result(self):
        """Blocks until service has returned and returns result.

        Equivalent to a regular (synchronous) service call.
        """
        self.join()
        return self.result
