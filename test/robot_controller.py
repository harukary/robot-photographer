import rospy
import sys
sys.path.append('..')
from scripts.RobotController import TurtlebotController

controller = TurtlebotController("cmd_vel")

while not rospy.is_shutdown():
    controller.rotate()