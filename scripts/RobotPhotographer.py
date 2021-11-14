import rospy
import sys
sys.path.append('..')
from scripts.RobotController import TurtlebotController

from scripts.Waiting import Waiting
from scripts.Searching import Searching
from scripts.Aiming import Aiming

from scripts.ViewpointSelection import viewpoint_selection

class RobotPhitigrapher:
    def __init__(self) -> None:
        self.state = "waiting"
        self.rate = rospy.Rate(10)

        self.robot = TurtlebotController()

        self.waiting = Waiting()
        self.searching = Searching(self.robot)
        self.aiming = Aiming(self.robot)
    
    def run(self):
        while not rospy.is_shutdown():
            print(self.state)
            if self.state == "waiting":
                result = self.waiting.run() # command server
                if result == "received":
                    self.state = "searching"

            elif self.state == "searching":
                result, target = self.searching.run() # searching for the target
                if result == "found":
                    goal = viewpoint_selection(target, self.robot)
                    self.robot.send_goal(goal)
                    self.state = "approaching"
                # timeout

            elif self.state == "approaching":
                result = self.robot.get_state() #wait for navigation result
                if result == "reached":
                    self.state = "aiming"

            elif self.state == "aiming":
                # optimize composition through viewfinder
                result = self.aiming.run()
                if result == "shooted":
                    self.robot.shoot()
                    self.state = "waiting"
            else:
                rospy.logwarn("Error")
                pass
            
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('robot_photographer', anonymous=True)
    photographer = RobotPhitigrapher()
    try:
        photographer.run()
    except rospy.ROSInterruptException:
        pass