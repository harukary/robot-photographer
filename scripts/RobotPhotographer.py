import rospy
import math

from visualization_msgs.msg import Marker, MarkerArray

import sys
sys.path.append('.')
from RobotController import RobotController

from states import Waiting, Patrolling, Approaching, Resetting, Shooting

class RobotPhotographer:
    def __init__(self, topics, init_state):
        self.state = init_state
        self.rate = rospy.Rate(10)

        self.robot = RobotController(topics)

        self.waiting = Waiting(self.robot)
        self.patrolling = Patrolling(self.robot)
        self.approaching = Approaching(self.robot)
        self.resetting = Resetting(self.robot)
        self.shooting = Shooting(self.robot)

        self.registered_objects = []
        self.rviz_pub = rospy.Publisher('/rviz_objects', MarkerArray, queue_size=1)
        # shooted pose publisher
    
    def run(self):
        while not rospy.is_shutdown():
            print(self.state)
            if self.state == "waiting":
                result = self.waiting.run() # command server
                if result == "ready":
                    self.state = self.patrolling.transition()

            elif self.state == "patrolling":
                result, targets = self.patrolling.run() # searching for the target
                if targets is not None:
                    for obj in targets:
                        exist = False
                        for i,r_obj in enumerate(self.registered_objects):
                            d = math.sqrt((obj['p_xy'][0]-r_obj['p_xy'][0])**2+(obj['p_xy'][1]-r_obj['p_xy'][1])**2)
                            if d < 0.2:
                                exist = True
                                self.registered_objects[i] = r_obj
                        if not exist:
                            self.registered_objects.append(obj)
                if result == "found":
                    self.robot.stop()
                    self.state = self.approaching.transition()

            elif self.state == "approaching":
                result = self.approaching.run() #wait for navigation result
                if result == "reached":
                    self.state = self.shooting.transition()
                elif result == "lost":
                    self.state = self.resetting.transition()

            elif self.state == "shooting":
                result = self.shooting.run()
                if result == "shooted":
                    self.state = self.waiting.transition() # "patrolling" for continuous photographing
                    # shooted pose 

            elif self.state == "resetting":
                result = self.resetting.run()
                if result == "go_next":
                    self.state = self.patrolling.transition()
            else:
                rospy.logwarn("Error: undifined state")
                pass
            self.publish_rviz()
            self.rate.sleep()
    
    def publish_rviz(self):
        markerArray = MarkerArray()
        for obj in self.registered_objects:
            marker = Marker()
            marker.header.frame_id = "/velodyne"
            marker.type = marker.CYLINDER
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.5
            marker.color.a = 0.5
            if obj['class'] == 0: # person
                marker.color.r = 1.0
            elif obj['class'] != 0: # other
                marker.color.b = 1.0
            else:
                marker.color.g = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = obj['p_xy'][0]
            marker.pose.position.y = obj['p_xy'][0] 
            marker.pose.position.z = 0. 
            # We add the new marker to the MarkerArray, removing the oldest marker from it when necessary
            markerArray.markers.append(marker)
            # Renumber the marker IDs
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1
        # Publish the MarkerArray
        self.rviz_pub.publish(markerArray)