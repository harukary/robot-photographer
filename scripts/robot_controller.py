import rospy
import sys
sys.path.append('.')
from RobotController import RobotController

topics = {
    'twist' : '/cmd_vel',
    'image' : '/camera/rgb/image_raw',
    'scan'  : '/scan',
    'pose'  : '/odom',
    'nav_s' : '/move_base',
    'nav_r' : '/move_base/result',
    'obj'   : '/objects',
}

if __name__ == '__main__':
    robot = RobotController(topics)
    print("ready")
    # while not rospy.is_shutdown():
    #     robot.rotate()

    pose = [(-1.1430511475,-1.39947879314,0.0),(0.0,0.0,0.712479235583,0.701693194255)]
    robot.send_goal(pose)

    while not rospy.is_shutdown():
        state = robot.get_state()
        if state == 'reached':
            print("goal reached")
            robot.shoot()
            break