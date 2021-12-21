import rospy
from RobotPhotographer import RobotPhotographer

topics = {
    'twist' : '/cmd_vel',
    'image' : '/camera/rgb/image_raw',
    'pose'  : '/odom',
    'nav_s' : '/move_base',
    'nav_r' : '/move_base/result',
    'obj'   : 'objects',
}

if __name__ == '__main__':
    rospy.init_node('robot_photographer', anonymous=True)
    photographer = RobotPhotographer(topics)
    try:
        photographer.run()
    except rospy.ROSInterruptException:
        pass