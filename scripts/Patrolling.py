import rospy

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