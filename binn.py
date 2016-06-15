import rospy
import math
from std_msgs.msg import String
from zeabus_vision_srv.srv import Boom2_Srv
from zeabus_vision_srv.msg import Boom_Msg
from zeabus_vision_src.srv import Boom_Srv
from AIControl import AIControl

class BinnMission (object):

    def __init__ (self):
        print "Now do bin"
        self.aicontrol = AIControl()
        self.object = String('bin')

    def check_white_center (self):
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):

            srv_name = 'Vision_Service2'
            rospy.wait_for_service(srv_name)
            print 'service starts'
            self.detect = rospy.ServiceProxy(srv_name, Boom2_Srv)
            object_data = Boom_Msg()
            object_data = self.detect(self.object,self.req)
            object_data = object_data.data

            if object_data.appear :
                if self.aicontrol.is_center ([object_data.x,object_data.y],-50,50,-50,50):
                    print 'Center'
                    if object_data.value > 2000 : ### near ###
                        self.aicontrol.turn_yaw_relative(object_data.angle)
                        break
                    else :
                        self.aicontrol.drive_z (-2.8)
                else :
                    print 'Not Center'
                    vx = self.aicontrol.adjust ((object_data.x/100)/object_data.value, -0.4, -0.2, 0.2, 0.4)
                    vy = self.aicontrol.adjust ((object_data.y/100)/object_data.value, -0.4, -0.2, 0.2, 0.4)
                    self.aicontrol.drive([vx,vy,0,0,0,0])
            else :
                count -= 1
            rospy.sleep(0.25)

    def run (self, cover): # if cover = 1, uncover = 0
        print 'Go to bin'
        count = 50
        if cover == 1:
            self.req = String('orange')
        else :
            self.req = String('white')

        check_white_center()

        if cover == 1:
            ''' font gripper '''
            self.aicontrol.drive ([-1,0,0,0,0,0])
            rospy.sleep(1)
            self.aicontrol.drive_z (-4)

            while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):

                srv_name = 'Vision_Service1'
                rospy.wait_for_service(srv_name)
                print 'service starts'
                self.detect = rospy.ServiceProxy(srv_name, Boom_Srv)
                object_data = Boom_Msg()
                object_data = self.detect(String('cover'),String('pink'))
                object_data = object_data.data

                if object_data.appear :
                    vx = object_data.value/25000
                    vy = self.aicontrol.adjust ((object_data.y/100)/object_data.value, -0.15, -0.1, 0.1, 0.15)
                    vz = self.aicontrol.adjust ((object_data.x/100)/object_data.value, -0.20, -0.1, 0.1, 0.20)

                    if self.aicontrol.is_center ([object_data.x,object_data.y],-50,50,-50,50):
                        self.aicontrol.drive ([vx,0,0,0,0,0])
                        print 'Handle is Center'
                    else :
                        self.aicontrol.drive ([0,vy,vz,0,0,0])
                        print 'Handle is not Center'
                else :
                    count -= 1
                rospy.sleep(0.25)

            ## grap
            self.aicontrol.drive_z (-3.5)
            self.aicontrol.drive ([0,1,0,0,0,0])
            rospy.sleep(0.1)
            self.aicontrol.drive ([0,-1,0,0,0,0])
            rospy.sleep(0.1)
            check_white_center ()

        ## drop x2 times
        print 'drop marker yet'
        print 'bin complete'

if __name__ == '__main__':
    binn_mission = BinnMission()
    #command
    binn_mission.run(## 0 or 1##)
    print "finish bin"
