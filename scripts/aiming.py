import rospy
import sys
sys.path.append('..')
from scripts.RobotController import TurtlebotController
from scripts.Aiming import Aiming

rospy.init_node('aiming')

controller = TurtlebotController("cmd_vel")
aiming = Aiming(controller)

while not rospy.is_shutdown():
    result = aiming.run()
    if result == "shooted":
        print("target shooted")
        break