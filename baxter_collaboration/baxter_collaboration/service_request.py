import threading


class FinishedServiceRequest:

    finished = True

    def wait_result(self):
        pass


finished_request = FinishedServiceRequest()


class ServiceRequest(threading.Thread):

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
        self.join()
        return self.result
