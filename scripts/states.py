import rospy
from std_msgs.msg import Bool

class Waiting:
    def __init__(self, command_topic='command'):
        # for test
        self.command_sub = rospy.Subscriber(command_topic, Bool, self.command_callback)
        self.received = False
    
    def run(self):
        if self.received:
            return "received"
        else:
            return None
    
    def command_callback(self, msg):
        self.received = msg.data

class Patrolling:
    def __init__(self, robot):
        self.robot = robot
        self.detected = None
    
    def run(self):
        if self.detected:
            self.robot.stop()
            return "found", self.target
        else:
            self.robot.roomba_walk()
            self.detected, self.target = self.detect_person(self.robot.objects)
            return False

    # TODO: look for a person in objects
    def detect_person(self, msg):
        target = None
        return False, target

class Approaching:
    def __init__(self, robot):
        self.robot = robot

        self.result = None
    
    def run(self):
        if self.result == "reached":
            self.robot.stop()
            return "reached"

        elif self.result == "lost":
            return "lost"
        else:
            self.result = self.robot.approach_object(target='person')
            return "approaching"


class Resetting:
    def __init__(self, robot):
        self.robot = robot
    
    def run(self):
        return 'recovered' # 1st step: give up the target


class Shooting:
    def __init__(self, robot):
        self.robot = robot

        self.shooted = False

    def run(self):
        robot_pose, target_pose = self.robot.shoot()
        # TODO: add to SLAM map?
        if self.shooted:
            return "shooted"
        else:
            return None