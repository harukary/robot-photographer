import rospy
from std_msgs.msg import Bool

class Waiting:
    def __init__(self, robot, command_topic='command'):
        self.robot = robot
        # for test
        self.command_sub = rospy.Subscriber(command_topic, Bool, self.command_callback)
        self.init()

    def init(self):
        self.received = False
        self.wait_count = 0
    
    def transition(self):
        self.init()
        return "waiting"

    def run(self):
        # self.targets = self.robot.detect_person(target=15)
        # print self.wait_count
        if self.received or self.wait_count >= 100:
            self.robot.stop()
            return "ready"
        else:
            self.robot.rotate(0.3)
            self.wait_count += 1
            return None
    
    def command_callback(self, msg):
        self.received = msg.data

class Patrolling:
    def __init__(self, robot):
        self.robot = robot
        self.init()
    
    def init(self):
        self.detected = False

    def transition(self):
        self.init()
        return "patrolling"
    
    def run(self):
        if self.detected:
            # self.robot.stop()
            return "found", self.targets
        else:
            self.robot.roomba_walk() # TODO: implement
            # self.robot.rotate(0.2)
            self.targets = self.robot.detect_person(target=15)
            if self.targets:
                self.detected = True
            return "patrolling", None

class Approaching:
    def __init__(self, robot):
        self.robot = robot
        self.init()

    def init(self):
        self.result = None
        self.lost_count = 0 

    def transition(self):
        self.init()
        return "approaching"

    def run(self):
        if self.result == "lost":
            self.lost_count += 1
        if self.result == "reached":
            self.lost_count = 0
            self.robot.stop()
            return "reached"
        else:
            if self.lost_count == 100:
                return "lost"
            else:
                self.result = self.robot.approach_object(target=15)
                return "approaching"


class Resetting:
    def __init__(self, robot):
        self.robot = robot
        self.init()
    
    def init(self):
        self.recovered = False

    def transition(self):
        self.init()
        return "resetting"

    def run(self):
        return 'go_next' # 1st step: give up the target


class Shooting:
    def __init__(self, robot):
        self.robot = robot
        self.init()

    def init(self):
        self.shooted = False
    
    def transition(self):
        self.init()
        return "shooting"

    def run(self):
        robot_pose, target_pose = self.robot.shoot()
        self.shooted = True
        # TODO: add to SLAM map?
        if self.shooted:
            return "shooted"
        else:
            return None