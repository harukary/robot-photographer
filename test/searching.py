import rospy
import sys
sys.path.append('..')
from scripts.RobotController import TurtlebotController
from scripts.Searching import Searching

rospy.init_node('searching')

controller = TurtlebotController("cmd_vel")
searching = Searching(controller)

while not rospy.is_shutdown():
    result = searching.run()
    if result == "detected":
        print("target detected")
        break