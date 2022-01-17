import rospy
import ros_numpy
import actionlib
import datetime
import cv2
import numpy as np

import math

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
import sensor_msgs.point_cloud2
from sensor_msgs.msg import PointCloud2, CompressedImage
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from obstacle_detector.msg import Obstacles

from std_msgs.msg import Float32MultiArray # -> SSD object topic

import sys
sys.path.append('.')
from check_occupancy import OccupancyCheck

PATH = '/root/Desktop/'

class RobotController:
    # pass topic names
    def __init__(self, topics):
        self.pose = None
        self.scan = None
        self.state = None
        self.objects = []
        self.scan_objs = []
        self.bridge = CvBridge()
        cv2.namedWindow("camera", cv2.WINDOW_NORMAL)
        self.focal_length = 0.3
        self.img_W = 640
        self.img_H = 480
        self.FOV_W = 80
        self.FOV_H = 60

        self.occupancy_check = OccupancyCheck(True)

        # Sub
        self.pose_sub = rospy.Subscriber(topics['pose'], Odometry, self.pose_callback)
        # self.image_sub = rospy.Subscriber(topics['image'],Image,self.image_callback)
        self.image_sub = rospy.Subscriber(topics['image'],CompressedImage,self.compressedimage_callback)
        self.depth_sub = rospy.Subscriber(topics['depth'],PointCloud2,self.depth_callback)

        self.scan_sub = rospy.Subscriber(topics['scan'],LaserScan,self.scan_callback)
        # self.navres_sub = rospy.Subscriber(topics['nav_r'], MoveBaseActionResult, self.nav_callback)
        self.objects_sub = rospy.Subscriber(topics['obj'], Float32MultiArray, self.objects_callback)
        self.scan_obj_sub = rospy.Subscriber('/raw_obstacles', Obstacles, self.scan_obj_callback)

        self.boxes_sub = rospy.Subscriber(topics['box'], Float32MultiArray, self.boxes_callback)
        self.lands_sub = rospy.Subscriber(topics['land'], Float32MultiArray, self.lands_callback)

        # Pub
        self.twist_pub = rospy.Publisher(topics['twist'], Twist, queue_size=1)

        # Action client
        # self.client = actionlib.SimpleActionClient(topics['nav_s'], MoveBaseAction)
        # self.client.wait_for_server()

    # update pose
    def pose_callback(self, msg):
        self.pose = msg.pose.pose
        # print(self.pose)
    
    # update scan
    def scan_callback(self, msg):
        self.scan = msg
        occupancy_data = self.occupancy_check.execute(scan=self.scan, r_max=4)
        # for i,data in enumerate(occupancy_data):
        #     print(i,data)
    
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
        np_arr = np.fromstring(msg.data, np.uint8)
        self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        cv2.imshow('camera', self.cv_image)
        cv2.waitKey(2)
    
    def depth_callback(self, msg):
        pass
        # xyz_array = ros_numpy.point_cloud2.get_xyz_points(msg)
        # print(xyz_array)
        # for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
        #     pt_x = point[0]
        #     pt_y = point[1]
        #     pt_z = point[2]
        #     print(pt_x,pt_y,pt_z)
        #     break

    # obj['bb'][0] : ymin
    # obj['bb'][1] : xmin
    # obj['bb'][2] : ymax
    # obj['bb'][3] : xmax
    # update ssd objects
    def objects_callback(self, msg):
        yolov5_result = np.array(msg.data)
        # print(yolov5_result)
        
        self.objects = []
        objs = yolov5_result[1:].reshape(-1,5)
        for bb in objs:
            obj = {'class':int(bb[0]), 'bb':list(bb[1:])}
            if obj['class'] == 0:
                r_x = (obj['bb'][3]+obj['bb'][1]-1.)/2
                x_m = r_x*2*self.focal_length*math.tan(math.radians(self.FOV_W)/2)
                theta_w = -math.atan2(x_m,self.focal_length)
                if self.scan is not None:
                    r = np.mean(self.get_scan(math.degrees(theta_w),2.))
                    obj['position'] = (r*math.cos(-theta_w),r*math.sin(-theta_w))
                    print(obj['position'])
            self.objects.append(obj)
    def scan_obj_callback(self,msg):
        self.scan_objs = msg.circles 
    # update face boxs
    def boxes_callback(self, msg):
        box_result = np.array(msg.data)
        # print(box_result)
        
        self.faceboxes = []
        boxes = box_result.reshape(-1,5)
        for box in boxes:
            self.faceboxes.append({'box':list(box[0:4]), 'conf':float(box[4])})

    # update face lands
    def lands_callback(self, msg):
        land_result = np.array(msg.data)
        #print(land_result)
        
        self.facelands = []
        landmarks = land_result.reshape(-1,10)
        for landmark in landmarks:
            self.facelands.append({'land':list(landmark)})
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
    
    def get_scan(self, deg, range_deg=10.):
        scan_list = []
        incr = self.scan.angle_increment
        a_min = self.scan.angle_min
        for i,s in enumerate(self.scan.ranges):
            if s == np.inf:
                pass
            elif deg-range_deg/2 < math.degrees(i*incr+a_min) < deg+range_deg/2:
                scan_list.append(s)
        return scan_list

    def roomba_walk(self):
        print(self.scan)
        if self.scan is None:
            return
        # RANGE = 30
        # # print 'len:', len(self.scan.ranges)
        # incr = self.scan.angle_increment
        # a_min = self.scan.angle_min
        forward_left = self.get_scan(30.,10.)
        forward_right = self.get_scan(-30.,10.)
        # for i,s in enumerate(self.scan.ranges):
        #     if s == np.inf:
        #         pass
        #     elif RANGE-5 < math.degrees(i*incr+a_min) < RANGE+5:
        #         forward_left.append(s)
        #     elif -RANGE-5 < math.degrees(i*incr+a_min) < -RANGE+5:
        #         forward_right.append(s)
        left = np.mean(forward_left)
        right = np.mean(forward_right)
        # print 'range:', left, right

        if left > 2 and right >2:
            self.translate(0.2)
            print('go')
        elif left <= 2:
            self.rotate(-0.3)
            print('right')
        elif right <= 2:
            self.rotate(0.3)
            print('left')

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
    def shoot(self,target=0):
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