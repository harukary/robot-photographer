import rospy
import actionlib
import cv2

from geometry_msgs.msg import Twist, PoseStamped
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Float32MultiArray # -> SSD object topic

class RobotController:
    # pass topic names
    def __init__(self, topics):
        rospy.init_node('robot_controller')
        self.pose = None
        self.state = None
        self.bridge = CvBridge()
        cv2.namedWindow("camera", cv2.WINDOW_NORMAL)

        # Sub
        self.pose_sub = rospy.Subscriber(topics['pose'], PoseStamped, self.pose_callback)
        self.image_sub = rospy.Subscriber(topics['image'],Image,self.image_callback)
        # TODO: depth?
        self.scan_sub = rospy.Subscriber(topics['scan'],LaserScan,self.scan_callback)
        self.navres_sub = rospy.Subscriber(topics['nav_r'], MoveBaseActionResult, self.nav_callback)
        self.objects_sub = rospy.Subscriber(topics['obj'], Float32MultiArray, self.objects_callback)

        # Pub
        self.twist_pub = rospy.Publisher(topics['twist'], Twist, queue_size=1)

        # Action client
        self.client = actionlib.SimpleActionClient(topics['nav_s'], MoveBaseAction)
        self.client.wait_for_server()

    # update pose
    def pose_callback(self, msg):
        self.pose = msg
    
    # update scan
    def scan_callback(self, msg):
        self.scan = msg
    
    # update ssd objects
    def objects_callback(self, msg):
        self.objects = msg

    # # waiting for navigation result
    # def nav_callback(self, msg):
    #     if msg.status.status == 3 and self.state == 'moving':
    #         self.state = 'reached'

    # update image
    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        cv2.namedWindow("camera", cv2.WINDOW_NORMAL)
        image = cv2.resize(self.cv_image, (int(self.cv_image.shape[1]/2), int(self.cv_image.shape[0]/2)))
        cv2.imshow("camera", image)
        cv2.waitKey(1)

    # put 0 to twist
    def stop(self):
        msg = Twist() # 0
        self.twist_pub.publish(msg)

    # rotation in vel
    def rotate(self, vel=0.3):
        msg = Twist()
        msg.angular.z = vel
        self.twist_pub.publish(msg)

    # go forward or go back in vel
    def translate(self, vel=0.2):
        msg = Twist()
        msg.linear.x = vel
        self.twist_pub.publish(msg)
    
    # TODO: Roomba walk -> read scan data to decide twist
    def roomba_walk(self):
        pass
    
    # TODO: approaching -> read scan & objects data to decide twist
    def approach_object(self, target='person'):
        res = "reached" # "lost", "? m"
        return 

    # # send a navigation goal
    # def send_goal(self, goal):
    #     goal_msg = self.goal_pose(goal)
    #     self.client.send_goal(goal_msg)
    #     self.state = "moving"

    # # convert pose to navigation goal
    # def goal_pose(self, pose):
    #     goal_pose = MoveBaseGoal()
    #     goal_pose.target_pose.header.frame_id = 'map'
    #     goal_pose.target_pose.pose.position.x = pose[0][0]
    #     goal_pose.target_pose.pose.position.y = pose[0][1]
    #     goal_pose.target_pose.pose.orientation.x = pose[1][0]
    #     goal_pose.target_pose.pose.orientation.y = pose[1][1]
    #     goal_pose.target_pose.pose.orientation.z = pose[1][2]
    #     goal_pose.target_pose.pose.orientation.w = pose[1][3]
    #     return goal_pose
    
    # get robot state
    def get_state(self):
        return self.state
    
    # shoot a photo
    def shoot(self):
        now = rospy.get_rostime()
        cv2.imwrite(str(now.secs)+'.jpg', self.cv_image)
        target = None # TODO: get person from objects
        return self.pose, target