import json
import rospy


class Timer:

    def __init__(self, path=None):
        self.data = []
        self.init = None
        self.path = path

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

    def save(self):
        if self.path is not None:
            with open(self.path, 'w') as f:
                json.dump(self.data, f)
