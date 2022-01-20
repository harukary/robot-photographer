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
        self.targets = self.robot.detect_target(target=0)
        # print self.wait_count
        if self.received or self.wait_count >= 100:
            self.robot.stop()
            return "ready"
        else:
            self.robot.rotate(0.31415)
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
            # registered = self.robot.explore()
            self.robot.roomba_walk()
            # self.robot.rotate(0.2)
            self.targets = self.robot.detect_target(target=0)
            registered = self.targets
            if self.targets:
                self.detected = True
            return "patrolling", registered

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
        pass
        if self.result == "lost":
            self.lost_count += 1
        if self.result == "reached":
            self.lost_count = 0
            self.robot.stop()
            return "reached"
        else:
            if self.lost_count == 50:
                return "lost"
            else:
                self.result = self.robot.approach_object(target=0)
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
        self.count = 0
    
    def transition(self):
        self.init()
        return "shooting"

    def run(self):
        self.shooted, targets = self.robot.shoot()
        # TODO: add to SLAM map?
        print(self.shooted)
        if self.shooted:
            return "shooted", targets
        elif self.count > 50:
            print('take too much time!')
            return "reapproach", None
        else:
            self.count += 1
            return "shooting" , None