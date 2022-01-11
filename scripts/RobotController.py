import rospy
import actionlib
import datetime
import cv2
import numpy as np

import math

from geometry_msgs.msg import Twist, PoseStamped
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from sensor_msgs.msg import CompressedImage #Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Float32MultiArray # -> SSD object topic

PATH = '/root/Desktop/'

class RobotController:
    # pass topic names
    def __init__(self, topics):
        # rospy.init_node('robot_controller')
        self.pose = None
        self.scan = None
        self.state = None
        self.objects = []
        self.bridge = CvBridge()
        cv2.namedWindow("camera", cv2.WINDOW_NORMAL)

        # Sub
        self.pose_sub = rospy.Subscriber(topics['pose'], PoseStamped, self.pose_callback)
        # self.image_sub = rospy.Subscriber(topics['image'],Image,self.image_callback)
        self.image_sub = rospy.Subscriber(topics['image'],CompressedImage,self.compressedimage_callback)
        # TODO: depth?
        self.scan_sub = rospy.Subscriber(topics['scan'],LaserScan,self.scan_callback)
        # self.navres_sub = rospy.Subscriber(topics['nav_r'], MoveBaseActionResult, self.nav_callback)
        self.objects_sub = rospy.Subscriber(topics['obj'], Float32MultiArray, self.objects_callback)

        # Pub
        self.twist_pub = rospy.Publisher(topics['twist'], Twist, queue_size=1)

        # Action client
        # self.client = actionlib.SimpleActionClient(topics['nav_s'], MoveBaseAction)
        # self.client.wait_for_server()

    # update pose
    def pose_callback(self, msg):
        self.pose = msg
        print(self.pose)
    
    # update scan
    def scan_callback(self, msg):
        self.scan = msg
    
    # update ssd objects
    def objects_callback(self, msg):
        ssd_result = np.array(msg.data)
        # print(ssd_result)
        
        self.objects = []
        if len(ssd_result) == 5: # No objects detected [0, 0, 0, 0, 0]
            bb = ssd_result
            self.objects.append({'class':int(bb[0]), 'bb':list(bb[1:])})
        else:
            objs = ssd_result.reshape(-1,5)
            for bb in objs:
                if len(bb) != 5:
                    pass
                self.objects.append({'class':int(bb[0]), 'bb':list(bb[1:])})

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
    
    # update image
    def compressedimage_callback(self, msg):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(msg.data, np.uint8)
        # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        cv2.imshow('camera', self.cv_image)
        cv2.waitKey(2)

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
        
    def go_and_rotate(self, linear_vel=0.2, angular_vel=0.3):
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.twist_pub.publish(msg)
    
    # TODO: Roomba walk -> read scan data to decide twist
    def roomba_walk(self):
        if self.scan is None:
            return
        RANGE = 30
        # print 'len:', len(self.scan.ranges)
        incr = self.scan.angle_increment
        a_min = self.scan.angle_min
        forward_left = []
        forward_right = []
        for i,s in enumerate(self.scan.ranges):
            if s == np.inf:
                pass
            elif RANGE-5 < math.degrees(i*incr+a_min) < RANGE+5:
                forward_left.append(s)
            elif -RANGE-5 < math.degrees(i*incr+a_min) < -RANGE+5:
                forward_right.append(s)
        left = np.mean(forward_left)
        right = np.mean(forward_right)
        # print 'range:', left, right

        if left > 2 and right >2:
            self.translate(0.2)
            print 'go'
        elif left <= 2:
            self.rotate(-0.3)
            print 'right'
        elif right <= 2:
            self.rotate(0.3)
            print 'left'

    def detect_person(self, target=15):
        targets = []
        for obj in self.objects:
            if obj['class'] == target: #person:15, wheel_chair:2
                targets.append(obj)
        return targets
    
    # TODO: approaching -> read scan & objects data to decide twist
    def approach_object(self, target=15):
        res = 'approaching'
        lost = True
        ymin = 10000
        xmin = 10000
        ymax = 0
        xmax = 0
        for obj in self.objects:
            if obj['class'] == target:
                lost = False                
                if obj['bb'][0] < ymin: ymin = obj['bb'][0]
                if obj['bb'][1] < xmin: xmin = obj['bb'][1]
                if obj['bb'][2] > ymax: ymax = obj['bb'][2]
                if obj['bb'][3] > xmax: xmax = obj['bb'][3]
        # print(center)

        if lost:
            res = 'lost'
        else:
            center = ((xmin+xmax)/2,(ymin+ymax)/2)
            height = ymax - ymin
            if 0.45 < center[0] < 0.55:
                if 0.5 < height < 0.9:
                    self.stop()
                    res = 'reached'
                elif height <= 0.5:
                    self.translate(vel=0.3)
                elif height >= 0.9:
                    self.translate(vel=-0.3)
            elif center[0] < 0.5:
                self.go_and_rotate(linear_vel=0.3, angular_vel=0.2)
            else:
                self.go_and_rotate(linear_vel=0.3, angular_vel=-0.2)
        return res
        
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
    def shoot(self,target=15):
        # now = rospy.get_rostime()
        # cv2.imwrite(PATH+str(now.secs)+'.jpg', self.cv_image)
        ymin = 10000
        xmin = 10000
        ymax = 0
        xmax = 0
        for obj in self.objects:
            if obj['class'] == target:
                if obj['bb'][0] < ymin: ymin = obj['bb'][0]
                if obj['bb'][1] < xmin: xmin = obj['bb'][1]
                if obj['bb'][2] > ymax: ymax = obj['bb'][2]
                if obj['bb'][3] > xmax: xmax = obj['bb'][3]
        center = ((xmin+xmax)/2,(ymin+ymax)/2)
        # height = ymax - ymin
        good_pic = False
        if 0.45 < center[0] < 0.55 and 0.1 < ymin and ymax < 0.9:
            good_pic = True
            now = datetime.datetime.now()
            cv2.imwrite(PATH+'photo_' + now.strftime('%Y%m%d%H%M%S' + '.jpg'), self.cv_image)
        target = None # TODO: get person from objects for registering to map
        return good_pic, target