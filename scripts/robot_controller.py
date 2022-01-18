import rospy
import sys
sys.path.append('.')
from RobotController import RobotController

topics = {
    'twist' : '/diff_drive_controller/cmd_vel', #'/cmd_vel',
    'image' : '/camera0/compressed', #'/camera/rgb/image_raw',
    'depth' : 'velodyne_points', # '/camera/depth/image',
    'scan'  : '/scan',
    'pose'  : '/odom',
    'nav_s' : '/move_base',
    'nav_r' : '/move_base/result',
    'obj'   : '/yolov5_result',
    'box'   : '/face_box',
    'land'  : '/face_land'
}

if __name__ == '__main__':
    rospy.init_node('robot_controller', anonymous=True)
    robot = RobotController(topics)
    rate = rospy.Rate(10)

    print("ready")
    while not rospy.is_shutdown():
        robot.roomba_walk()
        rate.sleep()
              