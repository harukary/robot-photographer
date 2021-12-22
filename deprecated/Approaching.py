
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