import rospy
import actionlib

from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class TurtlebotController:
    def __init__(self, topic_name):
        rospy.init_node('turtlebot_controller')
        self.pose = None
        self.state = None

        self.pose_sub = rospy.Subscriber('odom', PoseStamped, self.pose_callback)
        self.twist_pub = rospy.Publisher(topic_name, Twist, queue_size=1)

        self.image_sub = None

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def pose_callback(self, msg):
        self.pose = msg
    
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
        pass