import rospy
from RobotController import RobotController

TWIST='cmd_vel'
IMAGE="/camera/rgb/image_raw"
ODOM='odom'
NAV='move_base'
NAV_RES='/move_base/result'

if __name__ == '__main__':
    robot = RobotController(TWIST,IMAGE,ODOM,NAV,NAV_RES)
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