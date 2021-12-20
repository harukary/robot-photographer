import rospy
from std_msgs.msg import String

class Aiming:
    def __init__(self, robot):
        # for test
        self.objects_sub = rospy.Subscriber('target_in_pix', String, self.objects_callback)
        self.robot = robot

        self.aimed = None
    
    def run(self):
        if self.aimed:
            self.shoot()
            return "shooted"
        else:
            return None
    
    def objects_callback(self, msg):
        self.aimed = self.aim(msg)

    def aim(self, msg):
        if msg.data == "center":
            self.robot.stop()
            return True
        elif msg.data == "left":
            self.robot.rotate(vel=0.3)
            return False
        elif msg.data == "right":
            self.robot.rotate(vel=-0.3)
            return False
    
    def shoot(self):
        print("take a photo")
        pass