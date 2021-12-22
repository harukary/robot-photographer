import rospy
import sys
sys.path.append('.')
from RobotPhotographer import RobotPhotographer

# /camera0/compressed
# /client_count
# /connected_clients
# /diagnostics
# /diff_drive_controller/cmd_vel
# /imu
# /reset_robot_pose
# /rosout
# /rosout_agg
# /ssd_image                                                    sensor_msgs/Image
# /ssd_result                                                   std_msgs/Float32MultiArray               [7.0, 0.44653773307800293, 0.5636197924613953, 0.5256360769271851, 0.6879552006721497]
# /scan                                                         sensor_msgs/LaserScan
# /velodyne_nodelet_manager/bond
# /velodyne_nodelet_manager_cloud/parameter_descriptions
# /velodyne_nodelet_manager_cloud/parameter_updates
# /velodyne_nodelet_manager_driver/parameter_descriptions
# /velodyne_nodelet_manager_driver/parameter_updates
# /velodyne_nodelet_manager_laserscan/parameter_descriptions
# /velodyne_nodelet_manager_laserscan/parameter_updates
# /velodyne_packets
# /velodyne_points                                              sensor_msgs/PointCloud2

topics = {
    'twist' : '/diff_drive_controller/cmd_vel', #'/cmd_vel',
    'image' : '/camera0/compressed', #'/camera/rgb/image_raw',
    'scan'  : '/scan',
    'pose'  : '/odom',
    'nav_s' : '/move_base',
    'nav_r' : '/move_base/result',
    'obj'   : '/objects',
}

if __name__ == '__main__':
    rospy.init_node('robot_photographer', anonymous=True)
    photographer = RobotPhotographer(topics)
    try:
        photographer.run()
    except rospy.ROSInterruptException:
        pass