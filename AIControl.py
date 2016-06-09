import rospy
import math
import tf
from geometry_msgs.msg import Twist,TwistStamped,Pose,PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
class AIControl():

    _instance = None
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(Singleton, cls).__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__ (self):
        self.err = 0.4
        self.pose = Pose()
        self.auv = [0,0,0,0,0,0]
        self.goal = [-999,-999,-999]
        self.command = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        rospy.Subscriber ('/zeabus/state', Odometry, self.set_position)
        self.turn_yaw_rel = rospy.Publisher('/fix/rel/yaw', Float64, queue_size = 10)
        self.turn_yaw_abs = rospy.Publisher('/fix/abs/yaw', Float64, queue_size = 10)
        self.depth = rospy.Publisher('/fix/abs/depth', Float64, queue_size = 10)

    ##### start set environment #####
    def listToTwist (self, list):
        temp = Twist()
        temp.linear.x = list[0]
        temp.linear.y = list[1]
        temp.linear.z = list[2]
        temp.angular.x = list[3]
        temp.angular.y = list[4]
        temp.angular.z = list[5]
        return temp

    def pub (self, tw):
        print "linear  x:%f y:%f z:%f"%(tw.linear.x,tw.linear.y,tw.linear.z)
        # print "angular x:%f y:%f z:%f"%(tw.angular.x,tw.angular.y,tw.angular.z)
        for i in xrange(3):
            self.command.publish(tw)
            rospy.sleep(0.05)

    def set_position (self, data):
        self.pose = data.pose.pose
        pose = data.pose.pose
        tmp = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        ang = tf.transformations.euler_from_quaternion(tmp)
        self.auv[0] = pose.position.x
        self.auv[1] = pose.position.y
        self.auv[2] = pose.position.z
        self.auv[3] = ang[0]
        self.auv[4] = ang[1]
        self.auv[5] = ang[2]
        # print self.auv

    def get_pose(self):
        return self.auv
    ##### end set environment #####

    ##### move auv command #####
    def drive (self, list):
        self.pub(self.listToTwist(list))

    def stop (self):
        self.pub(self.listToTwist([0,0,0,0,0,0]))

    def turn_yaw_relative (self, degree):
        rad = math.radians(degree)
        rad = Float64(rad)
        for i in xrange(3):
            print rad
            self.turn_yaw_rel.publish(rad)
            rospy.sleep(0.05)
        print 'turn_yaw_relative'

    def turn_yaw_absolute (self, degree):
        rad = math.radians (degree)
        rad = Float64(rad)
        for i in xrange(3):
            print rad
            self.turn_yaw_abs.publish(rad)
            rospy.sleep(0.05)
        print 'turn_yaw_absolute'

    def drive_z (self,z):
        zz = Float64(z)
        for i in xrange(3):
            print zz
            self.depth.publish(zz)
            rospy.sleep(0.2)

        while not rospy.is_shutdown() and not(self.auv[2] >= (z-self.err) and self.auv[2] <= (z+self.err)):
            pass

        self.stop ()
        for i in xrange(3):
            self.depth.publish(self.auv[2])
            rospy.sleep(0.2)
        print 'drive_z complete'
    ##### end move auv #####

    ##### Navigation function #####
    def distance(self,x,y):
        return sum(map(lambda x,y : (x-y) **2 ,x,y))**0.5

    def twopi(self,rad):
        if(rad<=0):
            return abs(rad)
        else:
            return 2*math.pi-rad

    def w_yaw(self,setyaw):
        degi=self.twopi(self.auv[5])
        degf=self.twopi(setyaw)
        diff=(degi-2*math.pi-degf,degi-degf,2*math.pi-degf+degi)
        diff=min(diff,key=abs)
        diff*=2
        if(diff>=0):
            return abs(diff)
        return -abs(diff)

    def drive_xy(self,x,y,bit):
        ### v : velocity
        print 'drive xy'
        v = [0,0,0,0,0,0]
        while not rospy.is_shutdown():
            print self.auv
            delta_y=abs(x-self.auv[1])
            delta_x=abs(y-self.auv[2])
            rad=abs(self.auv[5]-math.atan2(delta_x,delta_y))

            disnow = self.distance((x,y),(self.auv[0],self.auv[1]))
            v[0]=min(disnow,0.3)*bit

            if(disnow>=1):
                v[5]=self.w_yaw(self.delta_radians(x,y,bit))


            if(disnow <= self.err):
                self.stop()
                rospy.sleep(0.25)
                break
            self.drive(v)
            rospy.sleep(0.25)
        print 'Finish drive_xy'

    def turn_yaw(self,radians):
        self.turn_yaw_absolute(math.degrees(radians))

        while not rospy.is_shutdown() and not (self.auv[5]>=radians-self.err and self.auv[5]<=radians+self.err):
            pass

        self.stop()

    def delta_radians (self,x,y,bit):
        radians = math.atan2((x-self.auv[0])*bit,(y-self.auv[1])*bit)
        radians -= math.pi/2
        radians *= -1
        if(radians > math.pi):
            return radians - 2*math.pi
        if(radians < -math.pi):
            return radians + 2*math.pi
        return radians

    def w_yaw(self,setyaw):
        degi=self.twopi(self.auv[5])
        degf=self.twopi(setyaw)
        diff=(degi-2*math.pi-degf,degi-degf,2*math.pi-degf+degi)
        diff=min(diff,key=abs)
        diff*=2
        if(diff>=0):
            return abs(diff)
        return -abs(diff)

    def goto(self,x,y,z,bit):
        self.drive_z(z)
        radians = self.delta_radians(x,y,bit)
        print radians
        self.turn_yaw_relative(math.degrees(radians))
        print 'finish turn_yaw_relative'
        self.drive_xy(x,y,bit)
    ##### endNavigation function #####

    ##### image function #####
    def is_center (self, point, xmin, xmax, ymin, ymax):
        if (xmin <= point[0] and point[0] <= xmax) and (ymin <= point[1] and point[1] <= ymax):
            return True
        return False

    def adjust (self, value, m_min, m_max, p_min, p_max):
        if value > 0:
            if value > p_max : return p_max
            if value < p_min : return p_min
        elif value < 0:
            if value < m_min : return m_min
            if value > m_max : return m_max
        return value

    def is_fail(self,count):
        if count>0:
            return False
        return True

if __name__=='__main__':
    print 'AIControl'
    # rospy.init_node('ai_control', anonymous=True) #comment if run main.py
    aicontrol=AIControl()
    # aicontrol.goto(-3,-2,-1.5,-1)
