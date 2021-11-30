import rospy
from RobotController import TurtlebotController

if __name__ == '__main__':
    robot = TurtlebotController(twist_topic="cmd_vel", image_topic="/camera/rgb/image_raw")
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