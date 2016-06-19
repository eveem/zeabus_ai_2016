#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from zeabus_vision_srv.srv import Boom_Srv
from zeabus_vision_srv.msg import Boom_Msg
from AIControl import AIControl

class NavigateMission (object):

    def __init__ (self):
        print "Now do Navigation Channel"
        ## subscribe vision
        srv_name = 'Vision_Service1'
        rospy.wait_for_service(srv_name)
        print 'service starts'
        self.detect = rospy.ServiceProxy(srv_name, Boom_Srv)
        ## old vision
        self.aicontrol = AIControl()
        self.object = String('portal')
        self.target = String('yellow')

    def run (self):
        print 'run in navigate'
        count = 50
        object_data = Boom_Msg()

        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):

            object_data = self.detect(self.object,self.target)
            object_data = object_data.data

            print object_data

            if object_data.appear :
                if object_data.value > 2000 : ### near ###
                    print 'near'
                    vy = self.aicontrol.adjust ((object_data.y/100)/object_data.value, -0.25, -0.1, 0.1, 0.25)
                    vz = self.aicontrol.adjust ((object_data.x/100)/object_data.value, -0.30, -0.1, 0.1, 0.30)
                    bc = 20
                else : ### far ###
                    print 'far'
                    vy = self.aicontrol.adjust ((object_data.y/100)/object_data.value, -0.35, -0.1, 0.1, 0.35)
                    vz = self.aicontrol.adjust ((object_data.x/100)/object_data.value, -0.40, -0.1, 0.1, 0.40)
                    bc = 40

                if self.aicontrol.is_center([object_data.x,object_data.y],-bc,bc,-bc,bc) :
                    print 'center'
                    if object_data.value > 3000 : ### very near ###
                        break
                    else :
                        self.aicontrol.drive ([0.5/object_data.value,0,0,0,0,0])
                else :
                    self.aicontrol.drive ([0,vy,vz,0,0,0])
                    print 'not center'
                rospy.sleep (0.25)
            else :
                count -= 1
        ### end while ###

        print 'see portal'
        self.aicontrol.drive_z (-1)
        print 'drive style'
        ### style ###
        self.aicontrol.stop ()
        rospy.sleep (0.25)
        print 'finish navigation channel'

if __name__ == '__main__':
    navigate_mission = NavigationMission()
    navigate_mission.run()
    print "finish Navigation Channel"
