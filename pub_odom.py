#https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf
#http://wiki.ros.org/ja/navigation/Tutorials/RobotSetup/Odom

import roslib
import rospy
import tf
import tf.msg
from math import sin, cos, pi
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Imu

x=0.0
y=0.0
th=0.0
vx=0.0
vy=0.0
vth=0.0

class testNode():
    def __init__(self):
        self.sub = rospy.Subscriber('diff_drive_controller/cmd_vel', Twist, self.callback)
        self.imu = rospy.Subscriber('/imu', Imu, self.callback_imu)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.samplePublisher()

    def callback(self, twist_msg): 
        global x
        global y
        global th
        global vx
        global vy
        global vth
        vx = twist_msg.linear.x
        vy = twist_msg.linear.y
        #vth = twist_msg.angular.z

    def callback_imu(self, imu_msg):
        global vth
        vth = -imu_msg.angular_velocity.z

    def samplePublisher(self):
        #r=rospy.Rate(1.0)
        global last_time
        global x
        global y
        global th
        global vx
        global vy
        global vth

        while not rospy.is_shutdown():
            #compute odometry in a typical way given the velocities of the robot
            current_time = rospy.Time.now()

            dt = (current_time - last_time).to_sec()
            delta_x = (vx * cos(th) - vy * sin(th)) * dt
            delta_y = (vx * sin(th) + vy * cos(th)) * dt
            delta_th = vth * dt

            x += delta_x
            y += delta_y
            th += delta_th

            #print "x:" + str(x) + ", vx:" + str(vx)
            #print "y:" + str(y) + ", vy:" + str(vy)
            #print "th:" + str(th) + ", vth" + str(vth)

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

            # first, we'll publish the transform over tf
            self.odom_broadcaster.sendTransform(
                (x, y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

            # publish the message
            self.odom_pub.publish(odom)
            last_time = current_time

            #r.sleep()


if __name__ == '__main__':
    rospy.init_node('testNode')
    #time.sleep(3.0)
    last_time = rospy.Time.now()
    node = testNode()
    rospy.spin()
