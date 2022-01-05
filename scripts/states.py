import rospy
from std_msgs.msg import Bool

class Waiting:
    def __init__(self, command_topic='command'):
        # for test
        self.command_sub = rospy.Subscriber(command_topic, Bool, self.command_callback)
        self.received = False
    
    def transition(self):
        self.received = False
        return "waiting"

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
        self.detected = False
    
    def transition(self):
        self.detected = False
        return "patrolling"
    
    def run(self):
        if self.detected:
            # self.robot.stop()
            return "found", self.targets
        else:
            # self.robot.roomba_walk() # TODO: implement
            # self.robot.rotate(0.2)
            self.targets = self.detect_person(self.robot.objects)
            if self.targets:
                self.detected = True
            return "patrolling", None

    def detect_person(self, objects):
        targets = []
        for obj in objects:
            if obj['class'] == 15:
                targets.append(obj)

        return targets

class Approaching:
    def __init__(self, robot):
        self.robot = robot
        self.result = None

    def transition(self):
        self.result = False
        return "approaching"

    def run(self):
        if self.result == "reached":
            self.robot.stop()
            return "reached"

        elif self.result == "lost":
            return "lost"
        else:
            self.result = self.robot.approach_object(target=15)
            return "approaching"


class Resetting:
    def __init__(self, robot):
        self.robot = robot
        self.recovered = False

    def transition(self):
        self.recovered = False
        return "resetting"

    def run(self):
        return 'recovered' # 1st step: give up the target


class Shooting:
    def __init__(self, robot):
        self.robot = robot
        self.shooted = False
    
    def transition(self):
        self.shooted = False
        return "shooting"

    def run(self):
        robot_pose, target_pose = self.robot.shoot()
        self.shooted = True
        # TODO: add to SLAM map?
        if self.shooted:
            return "shooted"
        else:
            return None