#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
''' vision
from zeabus_vision_srv.srv import ____
from zeabus_vision_srv.msg import ____
'''
from AIControl import AIControl

class SettMission (object):

    def __init__ (self):
        print "Now do Set Course"
        # subscribe vision
        self.aicontrol = AIControl()

    def find_fire (self, character):
        #while not found character
            #move to find
        #if small
            #while not center
                #move to center
            #if cover
                #grap
                #right
                #drop
                #left
        #else
            #while not center
                #move to center
        #fire

    def run (self, search):
        for i in xrange(2):
            find_fire (search[i])

if __name__ == '__main__':
    sett_mission = SettMission()
    #command
    sett_mission.run(['N','W'])
    print "finish bin"
