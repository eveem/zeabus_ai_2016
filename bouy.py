import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist,TwistStamped,Pose,PoseStamped
from zeabus_vision_srv.srv import Boom_Srv
from zeabus_vision_srv.msg import Boom_Msg
from AIControl import AIControl

class BouyMission (object):

    def __init__ (self):
        print "Now do bouy"
        ## subscribe vision
        srv_name = 'Vision_Service1'
        rospy.wait_for_service(srv_name)
        print 'service starts'
        self.detect = rospy.ServiceProxy(srv_name, Boom_Srv)
        ## old vision
        self.aicontrol = AIControl()
        self.object = String('bouy')
        self.target = ['red','red']
        self.first_point = Pose()
        self.point = Pose()

    def red_then_green (self):
        self.first_point = self.aicontrol.get_pose()
        for i in xrange(2):
            print 'will hit ' + self.target[i]
            count = 50
            while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
                object_data = self.detect(self.object,String(self.target[i]))
                object_data = object_data.data
                b = 80
                if object_data.value > 20000 :
                    b = 100
                    print 'detect center by near value'

                if object_data.appear :
                    if self.aicontrol.is_center([object_data.x,object_data.y],-b,b,-b,b):
                        self.angle = object_data.angle
                        print 'Center'
                        break
                    else :
                        print 'Not Center'
                        count -= 0.5

                    if object_data.value > 20000 :
                        vy = self.aicontrol.adjust (object_data.x/100, -0.4, -0.1, 0.1, 0.4)
                        vz = self.aicontrol.adjust (object_data.y/100, -0.4, -0.1, 0.1, 0.4)
                        print 'near'
                    else :
                        vy = self.aicontrol.adjust (object_data.x/100, -0.5, -0.2, 0.2, 0.5)
                        vz = self.aicontrol.adjust (object_data.y/100, -0.5, -0.2, 0.2, 0.5)
                        print 'far'
                    self.aicontrol.drive([1,vy,vz,0,0,0])
                    rospy.sleep(0.25)
                    self.point = self.aicontrol.get_pose()
                else :
                    count -= 1
                rospy.sleep(0.25)
            #self.point = self.point.pose
            print 'backward'
            self.aicontrol.drive ([-1,0,0,0,0,0])
            rospy.sleep (2)
            print 'go to set point'
            self.aicontrol.goto (self.first_point.position.x,self.first_point.position.y,self.first_point.position.z,1)
            print 'finish ' + self.target[i]
        print 'finish 2 bouy'

    def yellow_bouy (self):
        print 'do yellow bouy'
            #while not found
                #move to find
            #while not center
                #move to center
            #up
            #45 down
            #forward
            #45 up
            #backward
            #forward
        print 'finish yellow bouy'

if __name__ == '__main__':
    bouy_mission = BouyMission()
    #command
    bouy_mission.red_then_green()
    bouy_mission.yellow_bouy()
    print "finish bouy"
