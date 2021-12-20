import rospy
from std_msgs.msg import Bool

class Waiting:
    def __init__(self):
        # for test
        self.command_sub = rospy.Subscriber('command', Bool, self.command_callback)
        self.received = False
    
    def run(self):
        if self.received:
            return "received"
        else:
            return None
    
    def command_callback(self, msg):
        self.received = msg.data