#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
''' vision
from zeabus_vision_srv.srv import ____
from zeabus_vision_srv.msg import ____
'''
from AIControl import AIControl

class GateMission (object):

    def __init__ (self):
        print "Now do gate"
        # subscribe vision
        self.aicontrol = AIControl()

    # def run_with_vision (self):
        #if straight
            #go
        #else
            #move to target

    def run_without_vision (self):
        # self.aicontrol.turn_yaw_relative(15)
        # self.aicontrol.drive ([1,0,0,0,0,0])
        self.aicontrol.goto(19.5,5,-1.5,1)
        # rospy.sleep(13) #change time to wait
        print "forward"
        self.aicontrol.stop()
        rospy.sleep(1)

if __name__ == '__main__':
    print 'start gate'
    gate_mission = GateMission()
    #command
    gate_mission.run_without_vision()
    print "finish gate"
