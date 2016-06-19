#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from zeabus_vision_srv.srv import Boom2_Srv
from zeabus_vision_srv.msg import Boom_Msg
from AIControl import AIControl

class PathMission (object):

    def __init__ (self):
        print "Mission : Path"
        ## subscribe vision
        srv_name = 'Vision_Service2'
        rospy.wait_for_service(srv_name)
        print 'service starts'
        self.detect = rospy.ServiceProxy(srv_name, Boom2_Srv)
        ## old vision
        self.aicontrol = AIControl()
        self.object = String('path')
        self.req = String('orange')
        self.angle=0

    def goto_path (self):

        print 'Go to Path'
        count = 50
        object_data = Boom_Msg()

        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            object_data = self.detect(self.object,self.req)
            object_data = object_data.data

            if object_data.appear :
                if self.aicontrol.is_center([object_data.x,object_data.y],-50,50,-50,50):
                    self.angle = object_data.angle
                    print 'Center'
                    break
                else :
                    print 'Not Center'
                    vx = self.aicontrol.adjust (object_data.x/100, -0.4, -0.2, 0.2, 0.4)
                    vy = self.aicontrol.adjust (object_data.y/100, -0.4, -0.2, 0.2, 0.4)
                    self.aicontrol.drive([vx,vy,0,0,0,0])
            else :
                count -= 1
            rospy.sleep(0.25)

        if self.aicontrol.is_fail(count) :
            print 'Find Path Fail'
            return False

        print 'Find Path Complete'
        return True

    def run(self):
        print 'Start Path Mission'
        if(self.goto_path()):
            self.aicontrol.turn_yaw_relative(self.angle)
            print 'Path finish'
        else :
            self.aicontrol.goto(20,5,-1.5)
            if(self.goto_path()):
                self.aicontrol.turn_yaw_relative(self.angle)
                print 'Path finish'
            else :
                print 'Path Fail'

if __name__ == '__main__':
    path_mission = PathMission()
    #command
    path_mission.run()
