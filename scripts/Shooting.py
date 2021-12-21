
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