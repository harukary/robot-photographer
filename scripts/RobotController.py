import rospy
import actionlib
import cv2

from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class TurtlebotController:
    def __init__(self, twist_topic='cmd_vel', image_topic="/camera/rgb/image_raw"):
        rospy.init_node('turtlebot_controller')
        self.pose = None
        self.state = None

        self.pose_sub = rospy.Subscriber('odom', PoseStamped, self.pose_callback)
        self.twist_pub = rospy.Publisher(twist_topic, Twist, queue_size=1)

        self.image_sub = rospy.Subscriber(image_topic,Image,self.image_callback)
        self.bridge = CvBridge()
        cv2.namedWindow("camera", cv2.WINDOW_NORMAL)

        self.navres_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.nav_callback)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def pose_callback(self, msg):
        self.pose = msg
    
    def nav_callback(self, msg):
        if msg.status.status == 3 and self.state == 'moving':
            self.state = 'reached'

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        cv2.namedWindow("camera", cv2.WINDOW_NORMAL)
        image = cv2.resize(self.cv_image, (int(self.cv_image.shape[1]/2), int(self.cv_image.shape[0]/2)))
        cv2.imshow("camera", image)
        cv2.waitKey(1)

    def nav_res_callback(self, msg):
        self.state = msg #

    def stop(self):
        msg = Twist()
        self.twist_pub.publish(msg)

    def rotate(self, vel=0.3):
        msg = Twist()
        msg.angular.z = vel
        self.twist_pub.publish(msg)

    def translate(self, vel=0.2):
        msg = Twist()
        msg.linear.x = vel
        self.twist_pub.publish(msg)
    
    def send_goal(self, goal):
        goal_msg = self.goal_pose(goal)
        self.client.send_goal(goal_msg)
        self.state = "moving"

    def goal_pose(self, pose):
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = pose[0][0]
        goal_pose.target_pose.pose.position.y = pose[0][1]
        goal_pose.target_pose.pose.orientation.x = pose[1][0]
        goal_pose.target_pose.pose.orientation.y = pose[1][1]
        goal_pose.target_pose.pose.orientation.z = pose[1][2]
        goal_pose.target_pose.pose.orientation.w = pose[1][3]
        return goal_pose
    
    def get_state(self):
        return self.state
    
    def shoot(self):
        now = rospy.get_rostime()
        cv2.imwrite(str(now.secs)+'.jpg', self.cv_image)