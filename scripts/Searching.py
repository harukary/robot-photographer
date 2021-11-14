import rospy
from std_msgs.msg import Bool

class Searching:
    def __init__(self, robot):
        # for test
        self.objects_sub = rospy.Subscriber('target_found', Bool, self.objects_callback)
        self.robot = robot

        self.detected = None
        self.target = None
    
    def run(self):
        if self.detected:
            self.robot.stop()
            return "found", self.target
        else:
            self.robot.rotate(vel=0.3)
            return False
    
    def objects_callback(self, msg):
        self.detected = self.detect(msg)

    def detect(self, msg):
        return msg.data