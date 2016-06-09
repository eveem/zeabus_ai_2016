import rospy
import math
from std_msgs.msg import String
from zeabus_vision_srv.srv import Boom2_Srv
from zeabus_vision_srv.msg import Boom_Msg
from AIControl import AIControl

class BinnMission (object):

    def __init__ (self):
        print "Now do bin"
        ## subscribe vision
        srv_name = 'Vision_Service2'
        rospy.wait_for_service(srv_name)
        print 'service starts'
        self.detect = rospy.ServiceProxy(srv_name, Boom2_Srv)
        ## old vision
        self.aicontrol = AIControl()
        self.object = String('bin')
        # self.req =

    def run (self, cover): # if cover = 1, uncover = 0
        print 'Go to bin'
        count = 50
        object_data = Boom_Msg()
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):

            object_data = self.detect(self.object,self.req)
            object_data = object_data.data

            if object_data.appear :
                if self.aicontrol.is_center ([object_data.x,object_data.y],-50,50,-50,50):
                    print 'Center'
                    if object_data.value > 2000 : ### near ###
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

        if cover == 1:
            ## grap
            self.aicontrol.drive_z (-2)           ### up -> open binn ###
            self.aicontrol.drive ([0,1,0,0,0,0])  ### move -> drop cover ###
            rospy.sleep(0.1)
            ## drop cover
            self.aicontrol.drive ([0,-1,0,0,0,0]) ### move back to above bin ###
            rospy.sleep(0.1)
            self.aicontrol.drive_z (-2.8)

        ## drop x2 times

if __name__ == '__main__':
    binn_mission = BinnMission()
    #command
    binn_mission.run(## 0 or 1##)
    print "finish bin"
