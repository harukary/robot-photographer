import rospy

import sys
sys.path.append('.')
from RobotController import RobotController

from states import Waiting, Patrolling, Approaching, Resetting, Shooting

class RobotPhotographer:
    def __init__(self, topics):
        self.state = "waiting"
        self.rate = rospy.Rate(10)

        self.robot = RobotController(topics)

        self.waiting = Waiting()
        self.patrolling = Patrolling(self.robot)
        self.approaching = Approaching(self.robot)
        self.resetting = Resetting(self.robot)
        self.shooting = Shooting(self.robot)
    
    def run(self):
        while not rospy.is_shutdown():
            print(self.state)
            if self.state == "waiting":
                result = self.waiting.run() # command server
                if result == "received":
                    self.state = self.patrolling.transition()

            elif self.state == "patrolling":
                result, target = self.patrolling.run() # searching for the target
                if result == "found":
                    self.state = self.approaching.transition()

            elif self.state == "approaching":
                result = self.approaching.run() #wait for navigation result
                if result == "reached":
                    self.state = self.shooting.transition()
                elif result == "lost":
                    self.state = self.resetting.transition()

            elif self.state == "shooting":
                result = self.shooting.run()
                if result == "shooted":
                    self.state = self.waiting.transition() # "patrolling" for continuous photographing
            
            elif self.state == "resetting":
                result = self.resetting.run()
                if result == "recovered":
                    self.state = self.approaching.transition()
            else:
                rospy.logwarn("Error: undifined state")
                pass
            
            self.rate.sleep()