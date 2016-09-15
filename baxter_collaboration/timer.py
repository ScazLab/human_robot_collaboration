import rospy


class Timer:

    def __init__(self):
        self.data = []
        self.init = None
        self.start()

    def start(self):
        self.init = rospy.Time.now()
        self.log('started')

    def log(self, value):
        time = rospy.Time.now()
        self.data.append({
            'timestamp': str(time),
            'elapsed': (time - self.init).to_sec(),
            'value': value,
            })
