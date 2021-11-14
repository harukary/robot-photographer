import rospy
import sys
sys.path.append('..')
from scripts.Waiting import Waiting

rospy.init_node('waiting')

waiting = Waiting()

while not rospy.is_shutdown():
    result = waiting.run()
    if result == "received":
        print("command received")
        break