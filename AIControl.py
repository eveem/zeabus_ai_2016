import rospy
import math
import tf
from geometry_msgs.msg import Twist,TwistStamped,Pose,PoseStamped
from nav_msgs.msg import Odometrt
#from AISup import AISup

class AIControl():

    _instance = None
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(Singleton, cls).__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__ (self):

    def listToTwist (self, list):
        temp = Twist()
        temp.linear.x = list[0]
        temp.linear.y = list[1]
        temp.linear.z = list[2]
        temp.angular.x = list[3]
        temp.angular.y = list[4]
        temp.angular.z = list[5]
        return temp

    def drive (self, list):
        self.pub(self.listToTwist(list))

    def pub (self, tw):
        print "linear  x:%f y:%f z:%f"%(tw.twist.linear.x,tw.twist.linear.y,tw.twist.linear.z)
        print "angular x:%f y:%f z:%f"%(tw.twist.angular.x,tw.twist.angular.y,tw.twist.angular.z)
        for i in xrange(3):
            self.this.publish(tw)
            rospy.sleep(0.05)

    def detect (self, object, mission):
